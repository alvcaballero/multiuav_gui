import { useEffect } from 'react';
import { useThree } from '@react-three/fiber';

// Listens for 'scene-screenshot' (dispatched by ScreenshotButton, outside the
// Canvas) and captures the current frame. Must live inside <Canvas> to access
// the live gl/scene/camera via useThree.
const SceneScreenshot = () => {
  const { gl, scene, camera } = useThree();

  useEffect(() => {
    const handler = () => {
      gl.render(scene, camera);
      const dataURL = gl.domElement.toDataURL('image/png');

      const link = document.createElement('a');
      link.download = `scene_screenshot_${Date.now()}.png`;
      link.href = dataURL;
      link.click();
    };

    window.addEventListener('scene-screenshot', handler);
    return () => window.removeEventListener('scene-screenshot', handler);
  }, [gl, scene, camera]);

  return null;
};

export default SceneScreenshot;
