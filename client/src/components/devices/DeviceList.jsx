import { useEffect, useState } from 'react';
import { useDispatch } from 'react-redux';
import { List } from 'react-window';
import { makeStyles } from 'tss-react/mui';

import { devicesActions } from '../../store';
import DeviceRow from './DeviceRow';
import { useEffectAsync } from '../../reactHelper';

const useStyles = makeStyles()((theme) => ({
  list: {
    maxHeight: '100%',
  },
  listInner: {
    position: 'relative',
    margin: theme.spacing(1.5, 0),
  },
}));

const DeviceList = ({ devices }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  const [time, setTime] = useState(() => Date.now());
  void time;

  useEffect(() => {
    const interval = setInterval(() => setTime(Date.now()), 60000);
    return () => {
      clearInterval(interval);
    };
  }, []);

  useEffectAsync(async () => {
    const response = await fetch('/api/devices');
    if (response.ok) {
      dispatch(devicesActions.refresh(await response.json()));
    } else {
      throw Error(await response.text());
    }
  }, []);

  return (
    <List
      className={classes.list}
      rowComponent={DeviceRow}
      rowCount={devices.length}
      rowHeight={72}
      rowProps={{ devices }}
      overscanCount={5}
    />
  );
};

export default DeviceList;
