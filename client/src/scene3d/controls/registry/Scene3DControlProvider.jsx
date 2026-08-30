import { createContext, useContext, useMemo, useRef, useReducer, useCallback } from 'react';

export const CORNERS = ['top-left', 'top-right', 'bottom-left', 'bottom-right'];

const CORNER_STYLE = {
  'top-left': { top: 8, left: 8, alignItems: 'flex-start' },
  'top-right': { top: 8, right: 8, alignItems: 'flex-end' },
  'bottom-left': { bottom: 8, left: 8, alignItems: 'flex-start' },
  'bottom-right': { bottom: 8, right: 8, alignItems: 'flex-end' },
};

const Scene3DControlContext = createContext(null);

// Anchors HTML controls to a corner of the 3D view and stacks them via flexbox,
// mirroring MapLibre's map.addControl(control, position) — no manual pixel offsets.
export function Scene3DControlProvider({ children }) {
  const cornersRef = useRef({});
  const [, forceMount] = useReducer((n) => n + 1, 0);

  const getCornerContainer = useCallback((corner) => cornersRef.current[corner] ?? null, []);
  // Lets useScene3DControl retry once the corner <div> refs exist after this Provider's first commit.
  const ensureMounted = useCallback(() => forceMount(), []);

  const value = useMemo(
    () => ({ getCornerContainer, ensureMounted }),
    [getCornerContainer, ensureMounted],
  );

  return (
    <Scene3DControlContext.Provider value={value}>
      {children}
      {CORNERS.map((corner) => (
        <div
          key={corner}
          ref={(ref) => {
            cornersRef.current[corner] = ref;
          }}
          style={{
            position: 'absolute',
            zIndex: 10,
            display: 'flex',
            flexDirection: 'column',
            gap: 8,
            pointerEvents: 'none',
            ...CORNER_STYLE[corner],
          }}
        />
      ))}
    </Scene3DControlContext.Provider>
  );
}

export function useScene3DControlContext() {
  const ctx = useContext(Scene3DControlContext);
  if (!ctx) {
    throw new Error('useScene3DControl debe usarse dentro de <Scene3DControlProvider>');
  }
  return ctx;
}
