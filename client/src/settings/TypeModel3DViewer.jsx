import React, { useMemo } from 'react';
import { Box, Typography } from '@mui/material';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Bounds } from '@react-three/drei';
import useModelLoader from '../scene3d/models/ModelLoader.jsx';

/**
 * Standalone GLB preview for an ElementType's `model3d` — deliberately NOT
 * the mission scene (`R3FCanvas.jsx`): no sky/water/terrain/map tiles, just
 * enough to eyeball the uploaded model. Not part of `scene3d/`'s
 * layer/control system, this is a catalog-editing utility, not a scene
 * layer.
 */
const Model = ({ typeId }) => {
  const { model } = useModelLoader(typeId);

  const clone = useMemo(() => (model ? model.scene.clone() : null), [model]);

  if (!clone) return null;
  return (
    <Bounds fit clip observe margin={1.3}>
      <primitive object={clone} />
    </Bounds>
  );
};

const TypeModel3DViewer = ({ item }) => {
  if (!item?.model3d) {
    return (
      <Typography variant="body2" color="text.secondary">
        Subí un modelo 3D primero para poder previsualizarlo.
      </Typography>
    );
  }

  return (
    <Box
      sx={{
        height: 320,
        border: '1px solid',
        borderColor: 'divider',
        borderRadius: 1,
        overflow: 'hidden',
      }}
    >
      <Canvas camera={{ fov: 40 }}>
        <ambientLight intensity={0.6} />
        <directionalLight position={[5, 8, 5]} intensity={1.5} />
        <directionalLight position={[-5, 3, -5]} intensity={0.5} />
        <Model typeId={item.id} />
        <OrbitControls makeDefault enableDamping dampingFactor={0.1} />
      </Canvas>
    </Box>
  );
};

export default TypeModel3DViewer;
