import { useEffect, useState } from 'react';

// Same cadence as the server's watchdog tick (TELEMETRY_CHECK_INTERVAL_MS): the rate does not
// change faster than that.
const POLL_MS = 2000;

/**
 * Rate, age and health of every slot of a device, as the server's telemetry watchdog measures
 * them (GET /api/devices/:id/telemetry): `{ [slot]: { health, hz, ageMs } }`. `health` is
 * 'ok' | 'stale' | 'missing'; `hz` and `ageMs` are null until known. `{}` while the device
 * sends nothing.
 */
export const useSlotTelemetry = (deviceId) => {
  const [telemetry, setTelemetry] = useState({});

  useEffect(() => {
    if (deviceId == null) return undefined;
    const controller = new AbortController();
    const load = () =>
      fetch(`/api/devices/${deviceId}/telemetry`, { signal: controller.signal })
        .then((response) => (response.ok ? response.json() : {}))
        .then(setTelemetry)
        .catch(() => {});
    load();
    const timer = setInterval(load, POLL_MS);
    return () => {
      clearInterval(timer);
      controller.abort();
    };
  }, [deviceId]);

  return telemetry;
};
