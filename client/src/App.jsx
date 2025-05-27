import React from "react";
import { Outlet, useNavigate } from "react-router-dom";
import makeStyles from "@mui/styles/makeStyles";
import { LinearProgress, useMediaQuery, useTheme } from "@mui/material";
import SocketController from "./SocketController";

import { useDispatch } from 'react-redux';

import {  geofencesActions } from './store';
import { useEffectAsync } from './reactHelper';

const useStyles = makeStyles(() => ({
  page: {
    flexGrow: 1,
    overflow: "auto",
  },
  menu: {
    zIndex: 4,
  },
}));

const App = () => {
  const classes = useStyles();
  const theme = useTheme();
  const dispatch = useDispatch();

  useEffectAsync(async () => {
      const response = await fetch('/api/geofences');
      if (response.ok) {
        dispatch(geofencesActions.refresh(await response.json()));
      } else {
        throw Error(await response.text());
      }
  }, []);


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
