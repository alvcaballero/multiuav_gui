import { useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';

const send = (socket, type, payload) => socket.send(JSON.stringify({ type, payload }));

/**
 * The raw messages of ONE slot of a device, as they arrived at the server (before any decode).
 *
 * Asks the server for them over the shared WebSocket (`raw:subscribe`) and stops when the slot
 * changes or the page goes away (`raw:unsubscribe`); if the socket drops first, the server
 * forgets the subscription by itself. Messages stay out of Redux on purpose: they arrive up to
 * 10 times a second and nothing else in the app wants them.
 *
 * `status`: 'idle' (no slot) · 'waiting' (subscribed, nothing yet) · 'streaming' ·
 * 'rejected' (see `reason`: not_inspectable | invalid | too_many).
 */
export const useRawMessage = (deviceId, slot) => {
  // The effect depends on this: the page can open before the socket does (reload), and a
  // reconnect swaps window.websocket for a new connection that has no subscription.
  const connected = useSelector((state) => state.session.socket);
  const key = slot ? `${deviceId}:${slot}` : null;

  // The state belongs to the slot it was received for, so changing slot starts clean without
  // resetting anything inside an effect.
  const [received, setReceived] = useState(null);
  const [paused, setPaused] = useState(false);
  const pausedRef = useRef(false);

  useEffect(() => {
    const socket = window.websocket;
    if (!slot || !socket || socket.readyState !== WebSocket.OPEN) return undefined;

    const sameTarget = (m) => String(m.deviceId) === String(deviceId) && m.slot === slot;

    const onMessage = (event) => {
      // Camera frames are binary; everything else is JSON.
      if (typeof event.data !== 'string') return;
      let data;
      try {
        data = JSON.parse(event.data);
      } catch {
        return;
      }
      if (data.rawStatus && sameTarget(data.rawStatus)) {
        if (!data.rawStatus.ok)
          setReceived({ key, status: 'rejected', reason: data.rawStatus.reason });
      } else if (data.raw && sameTarget(data.raw) && !pausedRef.current) {
        setReceived({ key, status: 'streaming', message: data.raw, receivedAt: Date.now() });
      }
    };

    socket.addEventListener('message', onMessage);
    send(socket, 'raw:subscribe', { deviceId: Number(deviceId), slot });
    return () => {
      socket.removeEventListener('message', onMessage);
      if (socket.readyState === WebSocket.OPEN) {
        send(socket, 'raw:unsubscribe', { deviceId: Number(deviceId), slot });
      }
    };
  }, [deviceId, slot, key, connected]);

  const togglePaused = () => {
    pausedRef.current = !pausedRef.current;
    setPaused(pausedRef.current);
  };

  const current = key && received?.key === key ? received : { status: slot ? 'waiting' : 'idle' };
  return { ...current, connected: Boolean(connected), paused, togglePaused };
};
