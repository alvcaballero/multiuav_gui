import React from 'react';
import { TextField, MenuItem, Grid } from '@mui/material';

export const DEFAULT_CIRCLE_GEOMETRY = {
  geometry_type: 'circle',
  dimensions: { radius: 1, height: 1 },
  yaw: 0,
};

export const DEFAULT_RECTANGLE_GEOMETRY = {
  geometry_type: 'rectangle',
  dimensions: { width: 1, length: 1, height: 1 },
  yaw: 0,
};

const toNumber = (raw) => {
  const num = Number(raw);
  return Number.isFinite(num) ? num : 0;
};

export const describeGeometry = (geometry) => {
  if (!geometry) return null;
  const { geometry_type: geometryType, dimensions } = geometry;
  if (geometryType === 'circle')
    return `circular, radio ${dimensions.radius}m, altura ${dimensions.height}m`;
  if (geometryType === 'rectangle') {
    return `rectangular, ${dimensions.width}×${dimensions.length}m, altura ${dimensions.height}m`;
  }
  return null;
};

/**
 * Controlled editor for an `attributes.geometry` object
 * ({ geometry_type: 'circle'|'rectangle', dimensions: {...}, yaw }).
 * Used both for an ElementType's default geometry and for a per-item override.
 */
const GeometryFields = ({ value, onChange }) => {
  const geometry = value || DEFAULT_CIRCLE_GEOMETRY;

  const handleTypeChange = (geometryType) => {
    onChange(geometryType === 'circle' ? DEFAULT_CIRCLE_GEOMETRY : DEFAULT_RECTANGLE_GEOMETRY);
  };

  const handleDimensionChange = (key, raw) => {
    onChange({ ...geometry, dimensions: { ...geometry.dimensions, [key]: toNumber(raw) } });
  };

  const handleYawChange = (raw) => {
    onChange({ ...geometry, yaw: toNumber(raw) });
  };

  return (
    <Grid container spacing={2}>
      <Grid size={{ xs: 12, sm: 4 }}>
        <TextField
          select
          fullWidth
          label="Forma"
          value={geometry.geometry_type}
          onChange={(event) => handleTypeChange(event.target.value)}
        >
          <MenuItem value="circle">Circular</MenuItem>
          <MenuItem value="rectangle">Rectangular</MenuItem>
        </TextField>
      </Grid>

      {geometry.geometry_type === 'circle' ? (
        <Grid size={{ xs: 12, sm: 4 }}>
          <TextField
            fullWidth
            type="number"
            label="Radio (m)"
            value={geometry.dimensions.radius ?? ''}
            onChange={(event) => handleDimensionChange('radius', event.target.value)}
          />
        </Grid>
      ) : (
        <>
          <Grid size={{ xs: 12, sm: 4 }}>
            <TextField
              fullWidth
              type="number"
              label="Ancho (m)"
              value={geometry.dimensions.width ?? ''}
              onChange={(event) => handleDimensionChange('width', event.target.value)}
            />
          </Grid>
          <Grid size={{ xs: 12, sm: 4 }}>
            <TextField
              fullWidth
              type="number"
              label="Largo (m)"
              value={geometry.dimensions.length ?? ''}
              onChange={(event) => handleDimensionChange('length', event.target.value)}
            />
          </Grid>
        </>
      )}

      <Grid size={{ xs: 12, sm: 4 }}>
        <TextField
          fullWidth
          type="number"
          label="Altura (m)"
          value={geometry.dimensions.height ?? ''}
          onChange={(event) => handleDimensionChange('height', event.target.value)}
        />
      </Grid>

      <Grid size={{ xs: 12, sm: 4 }}>
        <TextField
          fullWidth
          type="number"
          label="Rotación / yaw (°)"
          value={geometry.yaw ?? 0}
          onChange={(event) => handleYawChange(event.target.value)}
        />
      </Grid>
    </Grid>
  );
};

export default GeometryFields;
