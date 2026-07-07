import React, { useState } from 'react';
import { Alert, IconButton, LinearProgress } from '@mui/material';
import ReplayIcon from '@mui/icons-material/Replay';
import { useDispatch, useSelector } from 'react-redux';
import { useAsyncTask } from './reactHelper';
import { sessionActions } from './store';

const ServerProvider = ({ children }) => {
  const dispatch = useDispatch();

  const initialized = useSelector((state) => !!state.session.server);
  const [error, setError] = useState(null);

  useAsyncTask(async () => {
    if (!error) {
      try {
        const response = await fetch('/api/server');
        if (response.ok) {
          dispatch(sessionActions.updateServer(await response.json()));
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
        setError(error.message);
      }
    }
    // dispatch is injected internally by useAsyncTask, see reactHelper.js
  }, [error]); // eslint-disable-line @eslint-react/exhaustive-deps

  if (error) {
    return (
      <Alert
        severity="error"
        action={
          <IconButton color="inherit" size="small" onClick={() => setError(null)}>
            <ReplayIcon fontSize="inherit" />
          </IconButton>
        }
      >
        {error}
      </Alert>
    );
  }
  if (!initialized) {
    return <LinearProgress />;
  }
  return children;
};

export default ServerProvider;
