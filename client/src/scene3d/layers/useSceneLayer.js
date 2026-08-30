import { useEffect, useId } from 'react';
import { useScene3DLayerContext } from './Scene3DLayerProvider';

// Self-registers as a togglable layer under `title` and returns whether it
// should currently render. Multiple components can share a title and are
// hidden/shown together, just like several map.addLayer calls sharing the
// same metadata['traccar:title'] in the 2D map.
export default function useSceneLayer(title) {
  const id = useId();
  const { registerLayer, unregisterLayer, hidden } = useScene3DLayerContext();

  useEffect(() => {
    registerLayer(id, title);
    return () => unregisterLayer(id);
  }, [id, title, registerLayer, unregisterLayer]);

  return !hidden.includes(title);
}
