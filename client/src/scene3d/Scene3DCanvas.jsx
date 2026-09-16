import { useSelector } from 'react-redux';

import R3FCanvas from './core/R3FCanvas';
import R3FMission from './scene/R3FMission';
import R3DMarkers from './scene/R3DMarkers';
import R3FDevices from './scene/R3FDevices';
import SelectDevice3D from './scene/SelectDevice3D';
import SceneScreenshot from './scene/SceneScreenshot';
import DownloadYamlButton from './controls/DownloadYamlButton';
import ScreenshotButton from './controls/ScreenshotButton';
import Scene3DNavigationControl from './controls/Scene3DNavigationControl';
import Scene3DLayerSwitcher from './controls/Scene3DLayerSwitcher';
import { Scene3DControlProvider } from './controls/registry/Scene3DControlProvider';
import { Scene3DLayerProvider } from './layers/Scene3DLayerProvider';
import useSceneLayer from './layers/useSceneLayer';

// Registers each scene group as a togglable layer (Scene3DLayerSwitcher lists
// them automatically) — must run inside <Scene3DLayerProvider>.
const Scene3DSceneContent = ({ routes, sessionMarkers }) => {
  const missionVisible = useSceneLayer('Misión');
  const markersVisible = useSceneLayer('Elementos');
  const devicesVisible = useSceneLayer('Vehículos');
  const boundingBoxesVisible = useSceneLayer('Cajas delimitadoras');

  return (
    <>
      <R3FCanvas>
        {missionVisible && <R3FMission routes={routes} />}
        {markersVisible && (
          <R3DMarkers elements={sessionMarkers} showBoundingBoxes={boundingBoxesVisible} />
        )}
        {devicesVisible && <R3FDevices />}
        <SelectDevice3D />
        <SceneScreenshot />
      </R3FCanvas>
      <Scene3DNavigationControl />
      <DownloadYamlButton />
      <ScreenshotButton />
      <Scene3DLayerSwitcher />
    </>
  );
};

const Scene3DCanvas = ({ className, style }) => {
  const routes = useSelector((state) => state.mission.route);
  const sessionMarkers = useSelector((state) => state.session.markers);

  return (
    <div
      className={className}
      style={{
        display: 'flex',
        flexDirection: 'column',
        flex: 1,
        minHeight: 0,
        ...style,
      }}
    >
      <Scene3DControlProvider>
        <Scene3DLayerProvider>
          <Scene3DSceneContent routes={routes} sessionMarkers={sessionMarkers} />
        </Scene3DLayerProvider>
      </Scene3DControlProvider>
    </div>
  );
};

export default Scene3DCanvas;
