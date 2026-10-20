import Scene3DCanvas from '../../scene3d/Scene3DCanvas';
import R3FGhostTarget from '../../scene3d/scene/R3FGhostTarget';

/** 3D view of the stage, with the pending manual move. Its own module so DeviceStage can lazy-load three.js. */
const DeviceScene3D = ({ ghost }) => (
  <Scene3DCanvas style={{ height: '100%' }}>{ghost && <R3FGhostTarget {...ghost} />}</Scene3DCanvas>
);

export default DeviceScene3D;
