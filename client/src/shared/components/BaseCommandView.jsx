import React, { Fragment } from 'react';
import { useSelector } from 'react-redux';
import { TextField, FormControlLabel, Checkbox } from '@mui/material';
import SelectField from '../../shared/components/SelectField';
import useCommandAttributes from '../../shared/attributes/useCommandAttributes';

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

const BaseCommandView = ({ deviceId, item, setItem }) => {
  const textEnabled = useSelector((state) => state.session.server.textEnabled);

  const availableAttributes = useCommandAttributes();

  const attributes = item && item.type ? availableAttributes[item.type] || [] : [];

  return (
    <Fragment key="sdf">
      <SelectField
        value={item.type || ''}
        onChange={(e) => setItem({ ...item, type: e.target.value, attributes: {} })}
        endpoint={
          deviceId
            ? `/api/commands/types?${new URLSearchParams({
                deviceId,
              }).toString()}`
            : '/api/commands/types'
        }
        keyGetter={(it) => it.type}
        titleGetter={(it) => it.type}
        label={'Command type'}
        getItems={(items) => console.log(items)}
      />
      {attributes.map(({ key, name, type }) => {
        if (type === 'boolean') {
          return (
            <FormControlLabel
              key={key}
              control={
                <Checkbox
                  checked={item.attributes[key]}
                  onChange={(e) => {
                    const updateItem = {
                      ...item,
                      attributes: { ...item.attributes },
                    };
                    updateItem.attributes[key] = e.target.checked;
                    setItem(updateItem);
                  }}
                />
              }
              label={name}
            />
          );
        }
        if (type === 'datetime') {
          return (
            <TextField
              key={key}
              type="datetime-local"
              value={utcIsoToLocalInput(item.attributes[key])}
              onChange={(e) => {
                const updateItem = {
                  ...item,
                  attributes: { ...item.attributes },
                };
                // Store the UTC instant; the server's contract requires "...Z".
                updateItem.attributes[key] = localInputToUtcIso(e.target.value);
                setItem(updateItem);
              }}
              label={name}
              InputLabelProps={{ shrink: true }}
            />
          );
        }
        return (
          <TextField
            key={key}
            type={type === 'number' ? 'number' : 'text'}
            value={item.attributes[key]}
            onChange={(e) => {
              const updateItem = {
                ...item,
                attributes: { ...item.attributes },
              };
              updateItem.attributes[key] =
                type === 'number' ? Number(e.target.value) : e.target.value;
              setItem(updateItem);
            }}
            label={name}
          />
        );
      })}
      {textEnabled && (
        <FormControlLabel
          control={
            <Checkbox
              checked={item.textChannel}
              onChange={(event) => setItem({ ...item, textChannel: event.target.checked })}
            />
          }
          label={'commandSendSms'}
        />
      )}
    </Fragment>
  );
};

export default BaseCommandView;
