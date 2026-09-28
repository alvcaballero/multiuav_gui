import React from 'react';
import { TextField, Grid } from '@mui/material';

const toNumber = (raw) => {
  const num = Number(raw);
  return Number.isFinite(num) ? num : 0;
};

/**
 * Controlled editor for an item/base's orientation state — `azimFront`
 * (0-360, "which way the front faces"). Per-instance state, the single
 * source of truth for orientation — replaces the old
 * `attributes.geometry.yaw` and the legacy client-only `heading` field.
 */
const OrientationFields = ({ azimFront, onChangeAzimFront }) => (
  <Grid container spacing={2}>
    <Grid size={{ xs: 12, sm: 6 }}>
      <TextField
        fullWidth
        type="number"
        label="Orientación / azimut frente (°)"
        slotProps={{ htmlInput: { min: 0, max: 360, step: 1 } }}
        value={azimFront ?? 0}
        onChange={(event) =>
          onChangeAzimFront(Math.min(360, Math.max(0, toNumber(event.target.value))))
        }
      />
    </Grid>
  </Grid>
);

export default OrientationFields;
