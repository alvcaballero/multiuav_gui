import { useState } from 'react';
import { useAsyncTask } from '../../reactHelper';

/**
 * The device model `name` (GET /api/device-models/:model), or null while none is chosen, it loads,
 * or it is unknown: the forms read its capabilities to show only the settings the device has
 * (deviceIdentity.capabilityEnabled).
 */
const useDeviceModel = (name) => {
  const [model, setModel] = useState(null);

  useAsyncTask(
    async ({ signal }) => {
      setModel(null);
      if (!name) return;
      const response = await fetch(`/api/device-models/${encodeURIComponent(name)}`, { signal });
      if (response.ok) setModel(await response.json());
    },
    [name],
  );

  return model;
};

export default useDeviceModel;
