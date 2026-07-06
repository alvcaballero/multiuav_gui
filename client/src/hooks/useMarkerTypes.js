import { useState, useEffect } from 'react';

let cachedTypes = null;

export const useMarkerTypes = () => {
  const [types, setTypes] = useState(cachedTypes || []);
  const [loading, setLoading] = useState(!cachedTypes);

  useEffect(() => {
    if (cachedTypes) return;
    const controller = new AbortController();
    fetch('/api/markers/types', { signal: controller.signal })
      .then((res) => res.json())
      .then((data) => {
        cachedTypes = data;
        setTypes(data);
      })
      .catch(() => {})
      .finally(() => setLoading(false));
    return () => controller.abort();
  }, []);

  return { types, loading };
};
