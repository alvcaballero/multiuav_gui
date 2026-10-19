import React, { Fragment } from 'react';
import { useSelector } from 'react-redux';
import { TextField, FormControlLabel, Checkbox, MenuItem } from '@mui/material';

// The server enforces a UTC-ISO-with-Z contract for command dates
// (assertUtcIsoDate). The user picks a time in their LOCAL zone, so we store the
// UTC instant in state and render it back as local for the datetime-local input.
// A `datetime-local` value has no zone: the browser reads/writes it as local time.
const localInputToUtcIso = (localValue) => {
  if (!localValue) return '';
  const d = new Date(localValue); // parsed as local time
  return Number.isNaN(d.getTime()) ? '' : d.toISOString(); // -> "...Z"
};

const utcIsoToLocalInput = (utcIso) => {
  if (!utcIso) return '';
  const d = new Date(utcIso);
  if (Number.isNaN(d.getTime())) return '';
  // Shift the UTC instant into local wall-clock, then trim to "YYYY-MM-DDTHH:mm".
  const local = new Date(d.getTime() - d.getTimezoneOffset() * 60000);
  return local.toISOString().slice(0, 16);
};

const fieldLabel = ({ name, unit }) => (unit ? `${name} (${unit})` : name);

// Parameter fields of one command, editing `item.attributes`. `params` is the command's
// `params` as GET /api/commands/types resolved it for the device (command_schema).
// Values are kept as typed: the server validates and converts them, and rejects what is
// out of range, so an empty number field means "not sent" (its default), never 0.
const BaseCommandView = ({ item, setItem, params = [] }) => {
  const textEnabled = useSelector((state) => state.session.server.textEnabled);

  const update = (key, value) =>
    setItem({ ...item, attributes: { ...item.attributes, [key]: value } });

  return (
    <Fragment>
      {params.map((param) => {
        const { id, type, required, description } = param;
        const label = fieldLabel(param);
        if (type === 'boolean') {
          return (
            <FormControlLabel
              key={id}
              control={
                <Checkbox
                  checked={Boolean(item.attributes[id])}
                  onChange={(e) => update(id, e.target.checked)}
                />
              }
              label={label}
            />
          );
        }
        if (type === 'datetime') {
          return (
            <TextField
              key={id}
              type="datetime-local"
              value={utcIsoToLocalInput(item.attributes[id])}
              // Store the UTC instant; the server's contract requires "...Z".
              onChange={(e) => update(id, localInputToUtcIso(e.target.value))}
              label={label}
              required={required}
              helperText={description}
              slotProps={{ inputLabel: { shrink: true } }}
            />
          );
        }
        if (type === 'select') {
          return (
            <TextField
              key={id}
              select
              value={item.attributes[id] ?? ''}
              onChange={(e) => update(id, e.target.value)}
              label={label}
              required={required}
              helperText={description}
            >
              {param.options.map((option) => (
                <MenuItem key={option.value} value={option.value}>
                  {option.name ?? option.value}
                </MenuItem>
              ))}
            </TextField>
          );
        }
        return (
          <TextField
            key={id}
            type={type === 'number' ? 'number' : 'text'}
            value={item.attributes[id] ?? ''}
            onChange={(e) => update(id, e.target.value)}
            label={label}
            required={required}
            helperText={description}
            slotProps={{
              htmlInput:
                type === 'number'
                  ? {
                      min: param.min ?? undefined,
                      max: param.max ?? undefined,
                      step: param.step ?? 'any',
                    }
                  : undefined,
            }}
          />
        );
      })}
      {textEnabled && (
        <FormControlLabel
          control={
            <Checkbox
              checked={Boolean(item.textChannel)}
              onChange={(event) => setItem({ ...item, textChannel: event.target.checked })}
            />
          }
          label="Send SMS"
        />
      )}
    </Fragment>
  );
};

export default BaseCommandView;
