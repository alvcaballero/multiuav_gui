import { createContext } from 'react';

/**
 * What the device page asks the 3D scene to show: { deviceId, level, onStatus } while the lidar viewer is on,
 * null otherwise. A context (not props) because the scene draws the devices from deep inside Scene3DCanvas,
 * and the point cloud has to live in the group of the device it belongs to.
 */
export const LidarCloudContext = createContext(null);
