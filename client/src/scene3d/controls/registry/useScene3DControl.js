import { useEffect, useState } from 'react';
import { useScene3DControlContext } from './Scene3DControlProvider';

// Returns a stable DOM container mounted inside the requested corner, stacked
// automatically alongside other controls in that corner — analogous to what
// MapLibre does internally for map.addControl(control, position).
export default function useScene3DControl(corner = 'top-right') {
  const [container] = useState(() => document.createElement('div'));
  const { getCornerContainer, ensureMounted } = useScene3DControlContext();
  const [ready, setReady] = useState(false);

  useEffect(() => {
    const parent = getCornerContainer(corner);
    if (!parent) {
      ensureMounted();
      return undefined;
    }
    parent.appendChild(container);
    setReady(true);
    return () => {
      parent.removeChild(container);
    };
  }, [corner, container, getCornerContainer, ensureMounted]);

  return ready ? container : null;
}
