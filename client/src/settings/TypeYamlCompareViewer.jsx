import React, { useMemo, useRef, useState } from 'react';
import { Box, Button, Checkbox, FormControlLabel, Typography } from '@mui/material';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Bounds, Html } from '@react-three/drei';
import useModelLoader from '../scene3d/models/ModelLoader.jsx';
import DefinitionModelTree from '../scene3d/models/DefinitionModelTree.jsx';
import GeometryWireframe from '../scene3d/models/GeometryWireframe.jsx';
import ItemParametersFields from '../common/components/ItemParametersFields.jsx';

const SceneReferences = () => (
  <group>
    <gridHelper args={[20, 20, '#64748b', '#cbd5e1']} />
    <axesHelper args={[5]} />
    <mesh renderOrder={1}>
      <sphereGeometry args={[0.06, 16, 16]} />
      <meshBasicMaterial color="#334155" depthTest={false} depthWrite={false} />
    </mesh>
    {[
      { label: 'X', position: [5, 0, 0], color: '#dc2626' },
      { label: 'Y', position: [0, 5, 0], color: '#15803d' },
      { label: 'Z', position: [0, 0, 5], color: '#2563eb' },
      { label: '(0, 0, 0)', position: [0, 0, 0], color: '#334155' },
    ].map(({ label, position, color }) => (
      <Html key={label} position={position} style={{ pointerEvents: 'none' }}>
        <span
          style={{
            color,
            background: 'rgba(255, 255, 255, 0.85)',
            borderRadius: 3,
            padding: '2px 4px',
            fontSize: 12,
            fontWeight: 600,
            whiteSpace: 'nowrap',
          }}
        >
          {label}
        </span>
      </Html>
    ))}
  </group>
);

const GlbModel = ({ typeId }) => {
  const { model } = useModelLoader(typeId);
  const clone = useMemo(() => (model ? model.scene.clone() : null), [model]);

  if (!clone) return null;
  return <primitive object={clone} />;
};

// Both models share ONE <Bounds> and the same origin so the YAML-derived
// wireframe overlays the GLB. This makes scale, orientation, and coverage
// differences directly visible.
//
// Without a GLB (no `model3d` uploaded yet) there's nothing to share a frame
// with — just show the YAML-derived model on its own, fit to itself.
const CompareScene = ({ typeId, model, hasGlb, geometry, visibleLayers }) => (
  <Bounds fit clip observe margin={1.3}>
    {hasGlb && visibleLayers.glb && <GlbModel typeId={typeId} />}
    {visibleLayers.yaml && <DefinitionModelTree model={model} />}
    {geometry && visibleLayers.geometry && (
      <GeometryWireframe geometry={geometry} color="#2563eb" />
    )}
  </Bounds>
);

/**
 * "Comprobar YAML vs modelo 3D" — resolves the type's definition.yaml
 * (current editor content, saved or not) into a link tree server-side
 * (POST .../definition/model) and overlays it on the real GLB, at
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
  const [visibleLayers, setVisibleLayers] = useState({ glb: true, yaml: true, geometry: true });
  const [previewParameters, setPreviewParameters] = useState({});
  const requestSequenceRef = useRef(0);

  const hasGlb = Boolean(item?.model3d);
  const canCompare = Boolean(yamlContent);
  const numericParameterDefs = useMemo(
    () => (item.attributes?.parameterDefs || []).filter((def) => def.dataType === 'number'),
    [item.attributes?.parameterDefs],
  );
  const defaultPreviewParameters = useMemo(
    () =>
      Object.fromEntries(
        numericParameterDefs
          .filter((def) => typeof def.default === 'number' && Number.isFinite(def.default))
          .map((def) => [def.key, def.default]),
      ),
    [numericParameterDefs],
  );
  const numericParameterKeys = useMemo(
    () => new Set(numericParameterDefs.map((def) => def.key)),
    [numericParameterDefs],
  );
  const currentPreviewParameters = Object.fromEntries(
    Object.entries(previewParameters).filter(([key]) => numericParameterKeys.has(key)),
  );
  const effectivePreviewParameters = { ...defaultPreviewParameters, ...currentPreviewParameters };

  const handleCompare = async (overrides = effectivePreviewParameters) => {
    const requestId = ++requestSequenceRef.current;
    setStatus('loading');
    setError(null);
    try {
      const response = await fetch(`/api/markers/types/${item.id}/definition/model`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ content: yamlContent, overrides }),
      });
      const data = await response.json();
      if (requestId !== requestSequenceRef.current) return;
      if (!response.ok) {
        setError(data);
        setModel(null);
        setStatus('error');
        return;
      }
      setModel(data);
      setStatus('idle');
    } catch (err) {
      if (requestId !== requestSequenceRef.current) return;
      setError({ error: err.message });
      setStatus('error');
    }
  };

  const handlePreviewParametersChange = (nextParameters) => {
    const currentParameters = Object.fromEntries(
      Object.entries(nextParameters).filter(([key]) => numericParameterKeys.has(key)),
    );
    setPreviewParameters(currentParameters);
    handleCompare({ ...defaultPreviewParameters, ...currentParameters });
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
          onClick={() => handleCompare()}
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

      {numericParameterDefs.length > 0 && (
        <Box sx={{ border: '1px solid', borderColor: 'divider', borderRadius: 1, p: 2 }}>
          <ItemParametersFields
            parameterDefs={numericParameterDefs}
            attributes={effectivePreviewParameters}
            onChange={handlePreviewParametersChange}
            title="Parámetros de previsualización"
          />
        </Box>
      )}

      {model && (
        <>
          <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
            <FormControlLabel
              control={
                <Checkbox
                  size="small"
                  checked={visibleLayers.glb}
                  disabled={!hasGlb}
                  onChange={(event) =>
                    setVisibleLayers((current) => ({ ...current, glb: event.target.checked }))
                  }
                />
              }
              label="Modelo GLB real"
            />
            <FormControlLabel
              control={
                <Checkbox
                  size="small"
                  checked={visibleLayers.yaml}
                  onChange={(event) =>
                    setVisibleLayers((current) => ({ ...current, yaml: event.target.checked }))
                  }
                />
              }
              label="Derivado del YAML (naranja)"
            />
            <FormControlLabel
              control={
                <Checkbox
                  size="small"
                  checked={visibleLayers.geometry}
                  disabled={!item.attributes?.geometry}
                  onChange={(event) =>
                    setVisibleLayers((current) => ({ ...current, geometry: event.target.checked }))
                  }
                />
              }
              label="Geometría por defecto (azul)"
            />
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
            <Canvas camera={{ fov: 40, position: [6, 4, 6] }}>
              <ambientLight intensity={0.6} />
              <directionalLight position={[5, 8, 5]} intensity={1.5} />
              <directionalLight position={[-5, 3, -5]} intensity={0.5} />
              <SceneReferences />
              <CompareScene
                typeId={item.id}
                model={model}
                hasGlb={hasGlb}
                geometry={item.attributes?.geometry}
                visibleLayers={visibleLayers}
              />
              <OrbitControls makeDefault enableDamping dampingFactor={0.1} />
            </Canvas>
          </Box>
        </>
      )}
    </Box>
  );
};

export default TypeYamlCompareViewer;
