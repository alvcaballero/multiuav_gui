import { FormControl, InputLabel, MenuItem, Select } from '@mui/material';
import { useCallback, useEffect, useState } from 'react';
import { useAsyncTask } from '../../reactHelper';

const defaultKeyGetter = (item) => item.id;
const defaultTitleGetter = (item) => item.name;
const defaultGetItems = (item) => item;

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
  keyGetter = defaultKeyGetter,
  titleGetter = defaultTitleGetter,
  getItems = defaultGetItems,
  disabled = false,
}) => {
  const [fetchedItems, setFetchedItems] = useState(undefined);
  const items = endpoint ? fetchedItems : data;

  // Resolves the full item for the current `value` once `loadedItems` is available -
  // either by auto-selecting the first one, or by notifying the parent of the item
  // that was already selected externally (value set before this list finished loading).
  const resolveCurrentItem = useCallback(
    (loadedItems) => {
      if (!loadedItems) return;
      if (emptyValue == null && (value === null || value === undefined)) {
        if (loadedItems.length > 0) onChange({ target: { value: loadedItems[0] } });
        return;
      }
      if (value !== null && value !== undefined) {
        getItems(loadedItems[value]);
      }
    },
    [emptyValue, value, onChange, getItems],
  );

  // Fetching only depends on `endpoint` - callers often pass inline `onChange`/`getItems`,
  // and keying this off `resolveCurrentItem` would re-fetch on every parent render.
  useAsyncTask(async () => {
    if (endpoint) {
      const response = await fetch(endpoint);
      if (response.ok) {
        setFetchedItems(await response.json());
      } else {
        throw Error(await response.text());
      }
    }
  }, [endpoint]);

  useEffect(() => {
    if (typeof items !== 'undefined') {
      resolveCurrentItem(items);
    }
  }, [items, resolveCurrentItem]);

  if (items) {
    return (
      <FormControl fullWidth={fullWidth} disabled={disabled}>
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
