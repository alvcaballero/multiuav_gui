import React from 'react';
import { TextField, MenuItem, IconButton, Button, Grid, Box, Typography } from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';
import AddIcon from '@mui/icons-material/Add';

const RESERVED_KEYS = new Set(['geometry', 'parameterDefs']);

const DATA_TYPES = [
  { value: 'number', label: 'Número' },
  { value: 'string', label: 'Texto' },
  { value: 'boolean', label: 'Sí/No' },
  { value: 'enum', label: 'Lista (enum)' },
];

const emptyDef = () => ({
  key: '',
  label: '',
  dataType: 'number',
  unit: '',
  options: [],
  default: '',
});

// A TextField's onChange always hands back a string — cast it to match
// dataType so `default` is stored as the real type (a number stored as
// "240" still passes ItemAttributesSchema/ParameterDefSchema, since both
// accept string|number|boolean, but it's not what a `dataType: 'number'`
// consumer expects). Left as-is while unparseable/mid-edit (e.g. "-" or "")
// so typing isn't fought.
const castDefault = (raw, dataType) => {
  if (raw === '' || raw == null) return raw;
  if (dataType === 'number') {
    const num = Number(raw);
    return Number.isFinite(num) ? num : raw;
  }
  if (dataType === 'boolean') {
    if (raw === 'true') return true;
    if (raw === 'false') return false;
    return raw;
  }
  return raw;
};

/**
 * Form-builder for an ElementType's `parameterDefs` — the set of
 * configurable state parameters every item of that type will expose (e.g. a
 * wind turbine's `nacelle_heading_deg`/`operational_status`). This defines
 * the SCHEMA only; each item's own values are edited elsewhere
 * (ItemParametersFields.jsx).
 */
const TypeParameterSchemaEditor = ({ value, onChange }) => {
  const defs = value || [];

  const updateDef = (index, patch) => {
    const next = defs.map((def, i) => (i === index ? { ...def, ...patch } : def));
    onChange(next);
  };

  const addDef = () => onChange([...defs, emptyDef()]);
  const removeDef = (index) => onChange(defs.filter((_, i) => i !== index));

  const keyError = (def, index) => {
    if (!def.key) return null;
    if (RESERVED_KEYS.has(def.key)) return `"${def.key}" es una key reservada`;
    if (!/^[a-zA-Z_][a-zA-Z0-9_]*$/.test(def.key)) return 'Debe ser un identificador válido';
    if (defs.some((other, i) => i !== index && other.key === def.key)) return 'Key duplicada';
    return null;
  };

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      {defs.length === 0 && (
        <Typography variant="body2" color="text.secondary">
          Este tipo todavía no tiene parámetros configurables.
        </Typography>
      )}
      {defs.map((def, index) => {
        const error = keyError(def, index);
        return (
          <Box
            key={index}
            sx={{ border: '1px solid', borderColor: 'divider', borderRadius: 1, p: 2 }}
          >
            <Grid container spacing={2} alignItems="flex-start">
              <Grid size={{ xs: 12, sm: 3 }}>
                <TextField
                  fullWidth
                  required
                  label="Key"
                  value={def.key}
                  error={Boolean(error)}
                  helperText={error}
                  onChange={(event) => updateDef(index, { key: event.target.value.trim() })}
                />
              </Grid>
              <Grid size={{ xs: 12, sm: 3 }}>
                <TextField
                  fullWidth
                  required
                  label="Label"
                  value={def.label}
                  onChange={(event) => updateDef(index, { label: event.target.value })}
                />
              </Grid>
              <Grid size={{ xs: 12, sm: 2 }}>
                <TextField
                  select
                  fullWidth
                  label="Tipo de dato"
                  value={def.dataType}
                  onChange={(event) => {
                    const dataType = event.target.value;
                    updateDef(index, { dataType, default: castDefault(def.default, dataType) });
                  }}
                >
                  {DATA_TYPES.map((option) => (
                    <MenuItem key={option.value} value={option.value}>
                      {option.label}
                    </MenuItem>
                  ))}
                </TextField>
              </Grid>
              <Grid size={{ xs: 12, sm: 2 }}>
                <TextField
                  fullWidth
                  label="Unidad"
                  value={def.unit || ''}
                  onChange={(event) => updateDef(index, { unit: event.target.value })}
                />
              </Grid>
              <Grid size={{ xs: 12, sm: 2 }} sx={{ display: 'flex', justifyContent: 'flex-end' }}>
                <IconButton onClick={() => removeDef(index)} aria-label="Eliminar parámetro">
                  <DeleteIcon />
                </IconButton>
              </Grid>

              {def.dataType === 'enum' && (
                <Grid size={{ xs: 12, sm: 6 }}>
                  <TextField
                    fullWidth
                    required
                    label="Opciones (separadas por coma)"
                    helperText="Ej: parked, running, maintenance"
                    value={(def.options || []).join(', ')}
                    onChange={(event) =>
                      updateDef(index, {
                        options: event.target.value
                          .split(',')
                          .map((option) => option.trim())
                          .filter(Boolean),
                      })
                    }
                  />
                </Grid>
              )}

              <Grid size={{ xs: 12, sm: 6 }}>
                <TextField
                  fullWidth
                  label="Valor por defecto"
                  value={def.default ?? ''}
                  onChange={(event) => updateDef(index, { default: castDefault(event.target.value, def.dataType) })}
                />
              </Grid>
            </Grid>
          </Box>
        );
      })}
      <Box>
        <Button startIcon={<AddIcon />} onClick={addDef}>
          Agregar parámetro
        </Button>
      </Box>
    </Box>
  );
};

export default TypeParameterSchemaEditor;
