import { createContext, useContext, useMemo, useCallback, useState } from 'react';
import usePersistedState from '../../shared/usePersistedState';

const Scene3DLayerContext = createContext(null);

// Registry of togglable "layers" of the 3D scene — the 3D-view equivalent of
// MapSwitcher's metadata['traccar:title'] grouping. Scene components self-register
// with a title via useSceneLayer; a single menu (Scene3DLayerSwitcher) toggles them.
export function Scene3DLayerProvider({ children }) {
  const [layers, setLayers] = useState({}); // id -> title, for currently mounted layers
  const [hidden, setHidden] = usePersistedState('hiddenScene3DLayers', []);

  const registerLayer = useCallback((id, title) => {
    setLayers((prev) => (prev[id] === title ? prev : { ...prev, [id]: title }));
  }, []);

  const unregisterLayer = useCallback((id) => {
    setLayers((prev) => {
      if (!(id in prev)) return prev;
      const next = { ...prev };
      delete next[id];
      return next;
    });
  }, []);

  const value = useMemo(
    () => ({ layers, registerLayer, unregisterLayer, hidden, setHidden }),
    [layers, hidden, registerLayer, unregisterLayer, setHidden],
  );

  return <Scene3DLayerContext.Provider value={value}>{children}</Scene3DLayerContext.Provider>;
}

export function useScene3DLayerContext() {
  const ctx = useContext(Scene3DLayerContext);
  if (!ctx) {
    throw new Error('useSceneLayer debe usarse dentro de <Scene3DLayerProvider>');
  }
  return ctx;
}
