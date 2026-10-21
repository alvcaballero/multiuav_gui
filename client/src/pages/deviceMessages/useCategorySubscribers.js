import { useEffect, useState } from 'react';

/**
 * The messages a category declares it receives (GET /api/category/:category, its `subscribers`
 * block): `{ [slot]: { name, messageType } }`. `null` while loading.
 */
export const useCategorySubscribers = (categoryName) => {
  const [subscribers, setSubscribers] = useState(null);

  useEffect(() => {
    if (!categoryName) return undefined;
    const controller = new AbortController();
    fetch(`/api/category/${encodeURIComponent(categoryName)}`, { signal: controller.signal })
      .then((response) => (response.ok ? response.json() : null))
      .then((category) => setSubscribers(category?.subscribers ?? {}))
      .catch(() => {});
    return () => controller.abort();
  }, [categoryName]);

  return subscribers;
};
