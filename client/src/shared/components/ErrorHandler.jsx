import {
  Snackbar,
  Alert,
  Button,
  Link,
  Dialog,
  DialogContent,
  DialogContentText,
  DialogActions,
  Typography,
} from '@mui/material';
import { useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { usePrevious } from '../../reactHelper';
import { errorsActions } from '../../store';

const ErrorHandler = () => {
  const dispatch = useDispatch();

  const error = useSelector((state) => state.errors.errors.find(() => true));
  const cachedError = usePrevious(error);

  const message = error || cachedError;
  const multiline = message?.includes('\n');
  const displayMessage = multiline ? message.split('\n')[0] : message;

  const [expanded, setExpanded] = useState(false);
  useEffect(() => {
    console.log('ErrorHandler mounted');
    if (error && !expanded) {
      //setExpanded(true);
    }
  }, [error, expanded]);

  return (
    <>
      <Snackbar open={Boolean(error) && !expanded}>
        <Alert elevation={6} onClose={() => dispatch(errorsActions.pop())} severity="error" variant="filled">
          {displayMessage}
          {multiline && (
            <>
              {' | '}
              <Link color="inherit" href="#" onClick={() => setExpanded(true)}>
                {}
              </Link>
            </>
          )}
        </Alert>
      </Snackbar>
      <Dialog open={expanded} onClose={() => setExpanded(false)} maxWidth={false}>
        <DialogContent>
          <DialogContentText>
            <Typography variant="caption">
              <pre>{message}</pre>
            </Typography>
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setExpanded(false)} autoFocus>
            {}
          </Button>
        </DialogActions>
      </Dialog>
    </>
  );
};

export default ErrorHandler;
