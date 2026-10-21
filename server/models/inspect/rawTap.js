import { eventBus, EVENTS } from '../../common/eventBus.js';
import { logger } from '../../common/logger.js';
import { WS_RAW_INSPECT_INTERVAL_MS } from '../../config/config.js';

// The camera slot is binary JPEG and already has its own channel (CameraStreamSubscriber).
const NOT_INSPECTABLE_SLOTS = new Set(['camera']);
// A client inspects one message at a time today; the cap only stops a client from fanning out.
const MAX_SUBSCRIPTIONS_PER_CLIENT = 8;

/** Why a subscription was refused (the `reason` of the RAW_STATUS event). */
export const RAW_REJECT = Object.freeze({
  INVALID: 'invalid',
  NOT_INSPECTABLE: 'not_inspectable',
  TOO_MANY: 'too_many',
});

/** One client watching one slot of one device. */
class Subscription {
  constructor() {
    this.lastSentAt = 0;
    this.pending = null; // newest message dropped inside the window, sent when it closes
    this.timer = null;
  }

  clear() {
    clearTimeout(this.timer);
    this.timer = null;
    this.pending = null;
  }
}

/**
 * Inspector de mensajes raw: deja que un cliente WS mire, tal como llegaron del
 * transporte (antes del decode), los mensajes de un slot de un dispositivo.
 *
 * Las suscripciones son efímeras: viven mientras viva la conexión del cliente
 * (`removeClient` desde el close del socket) o hasta un `unsubscribe` explícito.
 *
 * Los ingestores (ros.js, MqttServer, FlatbufferServer) llaman a `publish()` en cada
 * mensaje, hasta 50 Hz por device. Sin suscriptores es una comparación de `size`. Con
 * ellos, cada cliente recibe como mucho un mensaje cada WS_RAW_INSPECT_INTERVAL_MS por
 * slot, y es el MÁS RECIENTE de la ventana: los intermedios se descartan, pero el último
 * nunca se pierde (un cambio de estado que llega justo después de otro mensaje del mismo
 * slot se envía al cerrar la ventana, no se tira).
 *
 * Los ingestores de ROS/MQTT ya están suscritos por el decode, así que "encender
 * la captura" es solo "hay alguien escuchando": no se abre ni se cierra ningún topic.
 *
 * Además guarda el ÚLTIMO mensaje de cada slot, haya o no suscriptores, y se lo manda al
 * cliente que se suscribe (`replayLast`, `cached: true` + `ageMs`). Sin eso, un slot por
 * evento (`vehicle_command_ack`, `threat`) no se podía inspeccionar: abrías la página y no
 * llegaba nada hasta el próximo evento. Es pisar una referencia en un Map (el mensaje ya
 * existe), así que cuesta lo mismo que antes; la memoria es un mensaje por device y slot.
 */
class RawTap {
  constructor() {
    // `${deviceId}:${slot}` → Map<clientId, Subscription>
    this._subs = new Map();
    // clientId → Set<key>, to count and to clean up without scanning every key
    this._byClient = new Map();
    // `${deviceId}:${slot}` → the newest message of that slot (same shape `publish` builds)
    this._last = new Map();
  }

  static _key(deviceId, slot) {
    return `${deviceId}:${slot}`;
  }

  /** @returns {{ok: true} | {ok: false, reason: string}} */
  subscribe(clientId, deviceId, slot) {
    if (!clientId || deviceId == null || !slot) return { ok: false, reason: RAW_REJECT.INVALID };
    if (NOT_INSPECTABLE_SLOTS.has(slot)) return { ok: false, reason: RAW_REJECT.NOT_INSPECTABLE };

    const key = RawTap._key(deviceId, slot);
    const owned = this._byClient.get(clientId) ?? new Set();
    if (!owned.has(key) && owned.size >= MAX_SUBSCRIPTIONS_PER_CLIENT) {
      return { ok: false, reason: RAW_REJECT.TOO_MANY };
    }

    if (!this._subs.has(key)) this._subs.set(key, new Map());
    this._subs.get(key).get(clientId)?.clear(); // subscribing again starts the window over
    this._subs.get(key).set(clientId, new Subscription());
    owned.add(key);
    this._byClient.set(clientId, owned);
    logger.debug('RawTap: subscribed', { clientId, key });
    return { ok: true };
  }

  /**
   * Sends the newest stored message of the slot to a client that just subscribed, marked
   * `cached` and with its age, so it is never mistaken for live data. Separate from
   * `subscribe` so the router can answer the subscribe (RAW_STATUS) first.
   */
  replayLast(clientId, deviceId, slot) {
    const key = RawTap._key(deviceId, slot);
    const sub = this._subs.get(key)?.get(clientId);
    const message = this._last.get(key);
    if (!sub || !message) return;
    this._send(clientId, sub, message, Date.now(), true);
  }

  unsubscribe(clientId, deviceId, slot) {
    this._drop(RawTap._key(deviceId, slot), clientId);
  }

  /** Baja al cliente de todas sus suscripciones (cierre del socket). */
  removeClient(clientId) {
    for (const key of [...(this._byClient.get(clientId) ?? [])]) this._drop(key, clientId);
  }

  _drop(key, clientId) {
    const clients = this._subs.get(key);
    clients?.get(clientId)?.clear();
    clients?.delete(clientId);
    if (clients?.size === 0) this._subs.delete(key);

    const owned = this._byClient.get(clientId);
    owned?.delete(key);
    if (owned?.size === 0) this._byClient.delete(clientId);
  }

  /**
   * Llamado por cada ingestor con el mensaje tal como llegó.
   * `raw` may be a function (robofleet: unpacking a buffer is not free): it runs at most
   * once per message, and only if some client is going to receive it.
   * @param {{deviceId: number|string, slot: string, transport: 'ros'|'mqtt'|'robofleet', raw: any|(() => any)}} msg
   */
  publish({ deviceId, slot, transport, raw }) {
    if (deviceId == null || !slot || NOT_INSPECTABLE_SLOTS.has(slot)) return;
    const key = RawTap._key(deviceId, slot);
    const now = Date.now();
    // Shared by every client (and the cache), so a lazy `raw` is resolved once however many
    // are watching. `ts` is when it ARRIVED, not when it is sent.
    const message = { deviceId, slot, transport, raw, ts: now, resolved: false, data: undefined };
    this._last.set(key, message);

    const clients = this._subs.get(key);
    if (!clients) return;

    for (const [clientId, sub] of clients) {
      const wait = sub.lastSentAt + WS_RAW_INSPECT_INTERVAL_MS - now;
      if (wait <= 0) {
        sub.clear();
        this._send(clientId, sub, message, now);
      } else {
        sub.pending = message; // newer than whatever was waiting
        sub.timer ??= setTimeout(() => this._flush(key, clientId), wait).unref();
      }
    }
  }

  _flush(key, clientId) {
    const sub = this._subs.get(key)?.get(clientId);
    if (!sub?.pending) return;
    const message = sub.pending;
    sub.clear();
    this._send(clientId, sub, message, Date.now());
  }

  _send(clientId, sub, message, now, cached = false) {
    if (!message.resolved) {
      message.data = typeof message.raw === 'function' ? this._resolve(message.raw) : message.raw;
      message.resolved = true;
      message.raw = undefined; // once resolved, the cache keeps the result and lets the buffer go
    }
    if (message.data === undefined) return;
    sub.lastSentAt = now;
    const { deviceId, slot, transport, ts, data } = message;
    // `ageMs` is computed here, not on the client: the two clocks need not agree.
    const extra = cached ? { cached: true, ageMs: now - ts } : {};
    eventBus.emitSafe(EVENTS.RAW_MESSAGE, { clientId, deviceId, slot, transport, ts, data, ...extra });
  }

  _resolve(thunk) {
    try {
      return thunk();
    } catch (err) {
      logger.warn(`RawTap: could not build the raw view: ${err.message}`);
      return undefined;
    }
  }
}

export const rawTap = new RawTap();
