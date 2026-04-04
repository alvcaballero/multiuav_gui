import { useState, useEffect } from 'react';

let cachedTypes = null;

export const useMarkerTypes = () => {
  const [types, setTypes] = useState(cachedTypes || []);
  const [loading, setLoading] = useState(!cachedTypes);

  useEffect(() => {
    if (cachedTypes) return;
    fetch('/api/markers/types')
      .then((res) => res.json())
      .then((data) => {
        cachedTypes = data;
        setTypes(data);
      })
      .catch(() => {})
      .finally(() => setLoading(false));
  }, []);

  return { types, loading };
};
