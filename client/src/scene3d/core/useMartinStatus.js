import { useState, useEffect } from 'react';

const MARTIN_HEALTH = `http://${window.location.hostname}:8080/catalog`;
const POLL_INTERVAL = 10_000; // re-check every 10s

/**
 * Returns true when the local Martin tile server is reachable.
 * Polls periodically so the 3D scene reacts if Martin starts/stops.
 */
const useMartinStatus = () => {
  const [available, setAvailable] = useState(false);

  useEffect(() => {
    let cancelled = false;

    const check = async () => {
      try {
        const res = await fetch(MARTIN_HEALTH, {
          method: 'HEAD',
          signal: AbortSignal.timeout(2000),
        });
        if (!cancelled) setAvailable(res.ok);
      } catch {
        if (!cancelled) setAvailable(false);
      }
    };

    check();
    const id = setInterval(check, POLL_INTERVAL);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  return available;
};

export default useMartinStatus;
