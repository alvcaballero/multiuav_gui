import React, { Fragment } from 'react';
import { useSelector } from 'react-redux';
import { TextField, FormControlLabel, Checkbox } from '@mui/material';
import SelectField from '../../shared/components/SelectField';
import useCommandAttributes from '../../shared/attributes/useCommandAttributes';

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
