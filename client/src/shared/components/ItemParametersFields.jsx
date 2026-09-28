import React from 'react';
import { TextField, MenuItem, FormControlLabel, Checkbox, Grid, Typography } from '@mui/material';

const toNumber = (raw) => {
  const num = Number(raw);
  return Number.isFinite(num) ? num : 0;
};

/**
 * Renders one input per parameter an ElementType defines (`parameterDefs`),
 * bound to the item/base's own `attributes` — the VALUES for those
 * parameters. The type owns the schema (what parameters exist, their data
 * type/options); the item only owns its values, never the schema itself.
 */
const ItemParametersFields = ({ parameterDefs, attributes, onChange }) => {
  if (!parameterDefs || parameterDefs.length === 0) return null;

  const setValue = (key, rawValue) => {
    onChange({ ...(attributes || {}), [key]: rawValue });
  };

  return (
    <Grid container spacing={2}>
      <Grid size={12}>
        <Typography variant="subtitle2">Parámetros</Typography>
      </Grid>
      {parameterDefs.map((def) => {
        const value = attributes?.[def.key] ?? def.default ?? '';
        const label = def.unit ? `${def.label} (${def.unit})` : def.label;

        if (def.dataType === 'enum') {
          return (
            <Grid size={{ xs: 12, sm: 6 }} key={def.key}>
              <TextField
                select
                fullWidth
                label={label}
                value={value}
                onChange={(event) => setValue(def.key, event.target.value)}
              >
                {(def.options || []).map((option) => (
                  <MenuItem key={option} value={option}>
                    {option}
                  </MenuItem>
                ))}
              </TextField>
            </Grid>
          );
        }

        if (def.dataType === 'boolean') {
          return (
            <Grid size={{ xs: 12, sm: 6 }} key={def.key}>
              <FormControlLabel
                control={
                  <Checkbox
                    checked={Boolean(value)}
                    onChange={(event) => setValue(def.key, event.target.checked)}
                  />
                }
                label={label}
              />
            </Grid>
          );
        }

        return (
          <Grid size={{ xs: 12, sm: 6 }} key={def.key}>
            <TextField
              fullWidth
              type={def.dataType === 'number' ? 'number' : 'text'}
              label={label}
              value={value}
              helperText={def.description}
              slotProps={
                def.dataType === 'number' && (def.min !== undefined || def.max !== undefined)
                  ? { htmlInput: { min: def.min, max: def.max } }
                  : undefined
              }
              onChange={(event) =>
                setValue(
                  def.key,
                  def.dataType === 'number' ? toNumber(event.target.value) : event.target.value,
                )
              }
            />
          </Grid>
        );
      })}
    </Grid>
  );
};

export default ItemParametersFields;
