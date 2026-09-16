import React from 'react';
import { Outlet } from 'react-router-dom';
import { makeStyles } from 'tss-react/mui';

import SocketController from './SocketController';

import { useDispatch } from 'react-redux';

import { geofencesActions } from './store';
import { useAsyncTask } from './reactHelper';

const useStyles = makeStyles()(() => ({
  page: {
    flexGrow: 1,
    overflow: 'auto',
  },
  menu: {
    zIndex: 4,
  },
}));

const App = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  useAsyncTask(async () => {
    const response = await fetch('/api/geofences');
    if (response.ok) {
      dispatch(geofencesActions.refresh(await response.json()));
    } else {
      throw Error(await response.text());
    }
    // dispatch is injected internally by useAsyncTask, see reactHelper.js
  }, []); // eslint-disable-line @eslint-react/exhaustive-deps

  return (
    <>
      <SocketController />
      <div className={classes.page}>
        <Outlet />
      </div>
    </>
  );
};

export default App;
