import { createPortal } from 'react-dom';
import useScene3DControl from './useScene3DControl';

// Wrap any control's JSX in this to anchor it to a corner of the 3D view and
// have it stack automatically with the other controls there — the 3D-view
// equivalent of map.addControl(control, position) in MapLibre.
const Scene3DControl = ({ corner = 'top-right', children }) => {
  const container = useScene3DControl(corner);
  if (!container) return null;
  return createPortal(<div style={{ pointerEvents: 'auto' }}>{children}</div>, container);
};

export default Scene3DControl;
