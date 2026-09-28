import React, { useEffect, useMemo, useState } from 'react';
import { Box, Button, Typography } from '@mui/material';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Bounds } from '@react-three/drei';
import * as THREE from 'three';
import useModelLoader from '../scene3d/models/ModelLoader.jsx';
import DefinitionModelTree from '../scene3d/models/DefinitionModelTree.jsx';

const GlbModel = ({ typeId, onSizeX }) => {
  const { model } = useModelLoader(typeId);
  const clone = useMemo(() => (model ? model.scene.clone() : null), [model]);

  useEffect(() => {
    if (!clone) return;
    const size = new THREE.Box3().setFromObject(clone).getSize(new THREE.Vector3());
    onSizeX?.(size.x);
  }, [clone, onSizeX]);

  if (!clone) return null;
  return <primitive object={clone} />;
};

// Both models share ONE <Bounds> instead of each getting its own — fitting
// them independently would make a tiny model and a huge one fill their own
// frame equally, which defeats "do these coincide in scale". The offset
// comes from the GLB's own measured bounding box, not a guessed constant,
// and the YAML-derived model only mounts once that size is known, so it
// never briefly overlaps the GLB at x=0 before settling into place.
//
// Without a GLB (no `model3d` uploaded yet) there's nothing to share a frame
// with — just show the YAML-derived model on its own, fit to itself.
const CompareScene = ({ typeId, model, hasGlb }) => {
  const [glbSizeX, setGlbSizeX] = useState(null);
  const offsetX = glbSizeX != null ? glbSizeX + Math.max(glbSizeX, 10) * 0.3 : 0;

  if (!hasGlb) {
    return (
      <Bounds fit clip observe margin={1.3}>
        <DefinitionModelTree model={model} />
      </Bounds>
    );
  }

  return (
    <Bounds fit clip observe margin={1.3}>
      <GlbModel typeId={typeId} onSizeX={setGlbSizeX} />
      {glbSizeX != null && <DefinitionModelTree model={model} position={[offsetX, 0, 0]} />}
    </Bounds>
  );
};

/**
 * "Comprobar YAML vs modelo 3D" — resolves the type's definition.yaml
 * (current editor content, saved or not) into a link tree server-side
 * (POST .../definition/model) and renders it next to the real GLB, at
 * matching relative scale, for a visual sanity check of whether they
 * coincide in shape/proportions. Not a numeric comparison (see the boolean-
 * subtraction volume check this is a first step towards) — this is "does it
 * look roughly right", nothing more.
 *
 * A `model3d` isn't required — without one this just shows the YAML-derived
 * model on its own, which is useful on its own (e.g. before anyone's
 * uploaded a GLB yet).
 */
const TypeYamlCompareViewer = ({ item, yamlContent }) => {
  const [model, setModel] = useState(null);
  const [status, setStatus] = useState('idle'); // idle | loading | error
  const [error, setError] = useState(null);

  const hasGlb = Boolean(item?.model3d);
  const canCompare = Boolean(yamlContent);

  const handleCompare = async () => {
    setStatus('loading');
    setError(null);
    try {
      const response = await fetch(`/api/markers/types/${item.id}/definition/model`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ content: yamlContent }),
      });
      const data = await response.json();
      if (!response.ok) {
        setError(data);
        setModel(null);
        setStatus('error');
        return;
      }
      setModel(data);
      setStatus('idle');
    } catch (err) {
      setError({ error: err.message });
      setStatus('error');
    }
  };

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      {!hasGlb && (
        <Typography variant="body2" color="text.secondary">
          Todavía no hay modelo 3D subido — esto solo va a mostrar lo que sale del YAML.
        </Typography>
      )}

      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
        <Button
          variant="contained"
          onClick={handleCompare}
          disabled={!canCompare || status === 'loading'}
        >
          {hasGlb ? 'Comprobar YAML vs modelo 3D' : 'Ver modelo derivado del YAML'}
        </Button>
        {status === 'loading' && (
          <Typography variant="body2" color="text.secondary">
            Resolviendo...
          </Typography>
        )}
      </Box>

      {error && (
        <Typography variant="body2" color="error.main">
          {error.link ? `Error en "${error.link}"` : 'Error'}
          {error.expression ? ` (expresión: ${error.expression})` : ''}: {error.error}
        </Typography>
      )}

      {model && (
        <>
          <Box sx={{ display: 'flex', gap: 2 }}>
            {hasGlb ? (
              <>
                <Typography variant="caption" color="text.secondary" sx={{ flex: 1 }}>
                  Izquierda: modelo GLB real
                </Typography>
                <Typography variant="caption" color="text.secondary" sx={{ flex: 1 }}>
                  Derecha: derivado del YAML (naranja, wireframe)
                </Typography>
              </>
            ) : (
              <Typography variant="caption" color="text.secondary">
                Derivado del YAML (naranja, wireframe) — sin modelo GLB para comparar todavía
              </Typography>
            )}
          </Box>
          <Box
            sx={{
              height: 360,
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
              <CompareScene typeId={item.id} model={model} hasGlb={hasGlb} />
              <OrbitControls makeDefault enableDamping dampingFactor={0.1} />
            </Canvas>
          </Box>
        </>
      )}
    </Box>
  );
};

export default TypeYamlCompareViewer;
