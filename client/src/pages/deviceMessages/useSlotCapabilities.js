import { useEffect, useState } from 'react';

/**
 * Which capability owns each telemetry slot (the capability catalog, GET /api/device-models/capabilities):
 * `{ [slot]: capability }`. A slot no capability owns is not in it. `null` while loading.
 */
export const useSlotCapabilities = () => {
  const [owners, setOwners] = useState(null);

  useEffect(() => {
    const controller = new AbortController();
    fetch('/api/device-models/capabilities', { signal: controller.signal })
      .then((response) => (response.ok ? response.json() : {}))
      .then((catalog) => {
        const map = {};
        for (const [capability, entry] of Object.entries(catalog ?? {})) {
          for (const slot of entry.slots ?? []) map[slot] = capability;
        }
        setOwners(map);
      })
      .catch(() => {});
    return () => controller.abort();
  }, []);

  return owners;
};
