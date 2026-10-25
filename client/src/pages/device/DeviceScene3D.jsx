import Scene3DCanvas from '../../scene3d/Scene3DCanvas';
import R3FGhostTarget from '../../scene3d/scene/R3FGhostTarget';
import { LidarCloudContext } from '../../scene3d/lidar/LidarCloudContext';

/**
 * 3D view of the stage, with the pending manual move and, while `lidar` ({ deviceId, level, onStatus })
 * is set, that device's point cloud. Its own module so DeviceStage can lazy-load three.js.
 */
const DeviceScene3D = ({ ghost, lidar }) => (
  <LidarCloudContext value={lidar ?? null}>
    <Scene3DCanvas style={{ height: '100%' }}>
      {ghost && <R3FGhostTarget {...ghost} />}
    </Scene3DCanvas>
  </LidarCloudContext>
);

export default DeviceScene3D;
