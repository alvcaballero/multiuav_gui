// Command encoders — qué endpoint de un device implementa cada command, y cómo se arma su
// mensaje. Mismo patrón que rosDecode: un registry, no un if-chain ni un switch por dispatch.
//
// Un command (Takeoff) es una acción de dominio; cada plataforma la implementa con SU mensaje
// (PX4: px4_msgs/msg/VehicleCommand; otro firmware: otro). Por eso un encoder se registra por
// (CommandType, endpoint), y el endpoint se reconoce por lo que la categoría ya declara en
// devices_msg.yaml — su key y su msgType:
//
//   { msgType }       msgType específico: dice por sí mismo qué hace (VehicleCommand, GimbalAction).
//   { key }           msgType genérico (std_srvs/SetBool, Trigger…): lo que distingue StopMission
//                     de PauseMission es la key del endpoint, con cualquier msgType.
//   { key, msgType }  los dos.
//
// Especificidad, de mayor a menor: (key, msgType) > (key, ANY) > (ANY, msgType).
//
// La disponibilidad se DERIVA: un device tiene un command si alguno de sus endpoints matchea un
// encoder de ese command. `commands.<Type>.via` en devices_msg.yaml solo hace falta cuando un
// command matchea más de un endpoint de la categoría: sin `via`, es un problema de boot.
//
// Un command `local` no toca el device (SaveHome): está siempre disponible y no tiene endpoint.

export const ANY = '*';

// Bloques invocables de devices_msg. Todos declaran su tipo en `messageType`.
const ENDPOINT_BLOCKS = ['services', 'actions', 'publishers'];

const keyOf = (key, msgType) => `${key}|${msgType}`;

/**
 * Los endpoints invocables de una categoría (services, actions, publishers; no subscribers).
 * @param {object} [categoryConfig] entrada de la categoría en devices_msg.yaml
 * @returns {{ block: string, key: string, msgType: string|undefined }[]}
 */
export function endpointsOf(categoryConfig) {
  return ENDPOINT_BLOCKS.flatMap((block) =>
    Object.entries(categoryConfig?.[block] ?? {}).map(([key, def]) => ({ block, key, msgType: def?.messageType }))
  );
}

/** `publishers.VehicleCommand` → { block, key } */
function parseVia(via) {
  const [block, ...rest] = String(via).split('.');
  return { block, key: rest.join('.') };
}

export function createCommandRegistry() {
  // type → Map(`key|msgType` → encode)
  const encoders = new Map();
  // type → run (commands locales)
  const locals = new Map();

  const forType = (type) => encoders.get(type) ?? encoders.set(type, new Map()).get(type);

  /**
   * @param {string} type                CommandType
   * @param {{ key?: string|string[], msgType?: string|string[] }} match al menos uno de los dos
   * @param {(ctx: object) => object} encode ({ type, attributes, position, endpoint }) → mensaje
   */
  function register(type, { key = ANY, msgType = ANY } = {}, encode) {
    if (key === ANY && msgType === ANY) {
      throw new Error(`commandEncoders: ${type} needs a key or a msgType to match`);
    }
    const table = forType(type);
    for (const k of [key].flat()) {
      for (const m of [msgType].flat()) {
        const id = keyOf(k, m);
        if (table.has(id)) throw new Error(`commandEncoders: duplicate encoder for ${type} ${id}`);
        table.set(id, encode);
      }
    }
  }

  /** Command que corre en el server sin endpoint (siempre disponible). */
  function registerLocal(type, run) {
    if (locals.has(type)) throw new Error(`commandEncoders: duplicate local command ${type}`);
    locals.set(type, run);
  }

  /** El encoder de `type` para un endpoint, por especificidad; undefined si no implementa el command. */
  function encoderFor(type, { key, msgType }) {
    const table = encoders.get(type);
    if (!table) return undefined;
    return table.get(keyOf(key, msgType)) ?? table.get(keyOf(key, ANY)) ?? table.get(keyOf(ANY, msgType));
  }

  /**
   * Qué commands expone una categoría y por qué endpoint. Un command que matchea varios
   * endpoints necesita `commands.<Type>.via`; si falta, o apunta a algo que no existe o no lo
   * implementa, va a `problems` y el command queda fuera.
   * @param {object} [categoryConfig]
   * @returns {{ commands: Map<string, object|null>, problems: string[] }} endpoint, o null si es local
   */
  function resolveCommands(categoryConfig) {
    const commands = new Map([...locals.keys()].map((type) => [type, null]));
    const problems = [];
    const endpoints = endpointsOf(categoryConfig);
    for (const type of encoders.keys()) {
      const matches = endpoints.filter((e) => encoderFor(type, e));
      const via = categoryConfig?.commands?.[type]?.via;
      if (via != null) {
        const { block, key } = parseVia(via);
        const endpoint = endpoints.find((e) => e.block === block && e.key === key);
        if (!endpoint) problems.push(`${type}: via '${via}' is not an endpoint of the category`);
        else if (!encoderFor(type, endpoint)) problems.push(`${type}: via '${via}' (${endpoint.msgType}) has no encoder`);
        else commands.set(type, endpoint);
        continue;
      }
      if (matches.length === 1) commands.set(type, matches[0]);
      else if (matches.length > 1) {
        const list = matches.map((e) => `${e.block}.${e.key}`).join(', ');
        problems.push(`${type}: matches ${list}; pick one with commands.${type}.via`);
      }
    }
    return { commands, problems };
  }

  /**
   * Arma el mensaje de un command para su endpoint resuelto.
   * @returns {object} el mensaje, listo para el transporte
   */
  function encode(type, endpoint, ctx = {}) {
    const fn = encoderFor(type, endpoint);
    if (!fn) throw new Error(`commandEncoders: no encoder for ${type} on ${endpoint.block}.${endpoint.key}`);
    return fn({ ...ctx, type, endpoint });
  }

  const localRun = (type) => locals.get(type);

  return { register, registerLocal, encoderFor, resolveCommands, encode, localRun };
}
