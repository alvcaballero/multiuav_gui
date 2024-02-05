import { FormControl, InputLabel, MenuItem, Select } from '@mui/material';
import React, { useEffect, useState } from 'react';
import { useEffectAsync } from '../../reactHelper';

const SelectField = ({
  label,
  fullWidth,
  multiple,
  value,
  emptyValue = 0,
  emptyTitle = '\u00a0',
  onChange,
  endpoint,
  data,
  keyGetter = (item) => item.id,
  titleGetter = (item) => item.name,
  getItems = (item) => item,
}) => {
  const [items, setItems] = useState(data);

  useEffectAsync(async () => {
    if (endpoint) {
      const response = await fetch(endpoint);
      if (response.ok) {
        setItems(await response.json());
      } else {
        throw Error(await response.text());
      }
    }
  }, []);

  useEffect(() => {
    if (typeof items !== 'undefined') {
      getItems(items[value]);
    }
  }, [items]);

  if (items) {
    return (
      <FormControl fullWidth={fullWidth}>
        <InputLabel>{label}</InputLabel>
        <Select
          label={label}
          multiple={multiple}
          value={value}
          onChange={(e) => onChange(e, items)}
        >
          {!multiple && emptyValue !== null && <MenuItem value={emptyValue}>{emptyTitle}</MenuItem>}
          {items.map((item) => (
            <MenuItem key={keyGetter(item)} value={keyGetter(item)}>
              {titleGetter(item)}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    );
  }
  return null;
};

export default SelectField;
