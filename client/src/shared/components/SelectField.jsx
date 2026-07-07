import { FormControl, InputLabel, MenuItem, Select } from '@mui/material';
import { useCallback, useEffect, useState } from 'react';
import { useAsyncTask } from '../../reactHelper';

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
  const [fetchedItems, setFetchedItems] = useState(undefined);
  const items = endpoint ? fetchedItems : data;

  const autoSelectFirstItem = useCallback(
    (loadedItems) => {
      if (
        emptyValue == null &&
        (value === null || value === undefined) &&
        loadedItems &&
        loadedItems.length > 0
      ) {
        onChange({ target: { value: loadedItems[0] } });
      }
    },
    [emptyValue, value, onChange],
  );

  useAsyncTask(async () => {
    if (endpoint) {
      const response = await fetch(endpoint);
      if (response.ok) {
        const loadedItems = await response.json();
        setFetchedItems(loadedItems);
        autoSelectFirstItem(loadedItems);
      } else {
        throw Error(await response.text());
      }
    }
  }, [endpoint, autoSelectFirstItem]);

  useEffect(() => {
    if (typeof items !== 'undefined' && value !== null) {
      getItems(items[value]);
    }
  }, [items, value, getItems]);

  useEffect(() => {
    if (!endpoint && typeof items !== 'undefined') {
      autoSelectFirstItem(items);
    }
  }, [endpoint, items, autoSelectFirstItem]);

  if (items) {
    return (
      <FormControl fullWidth={fullWidth}>
        <InputLabel>{label}</InputLabel>
        {value !== null && value !== undefined && (
          <Select
            label={label}
            multiple={multiple}
            value={value}
            onChange={(e) => onChange(e, items)}
          >
            {!multiple && emptyValue !== null && emptyValue !== undefined && (
              <MenuItem value={emptyValue}>{emptyTitle}</MenuItem>
            )}
            {items.map((item) => (
              <MenuItem key={keyGetter(item)} value={keyGetter(item)}>
                {titleGetter(item)}
              </MenuItem>
            ))}
          </Select>
        )}
      </FormControl>
    );
  }
  return null;
};

export default SelectField;
