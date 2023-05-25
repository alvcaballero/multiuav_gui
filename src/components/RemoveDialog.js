import React from 'react';
import Button from '@mui/material/Button';
import { Snackbar } from '@mui/material';
import makeStyles from '@mui/styles/makeStyles';


const useStyles = makeStyles((theme) => ({
  root: {
    [theme.breakpoints.down('md')]: {
      bottom: `calc(${theme.dimensions.bottomBarHeight}px + ${theme.spacing(1)})`,
    },
  },
  button: {
    height: 'auto',
    marginTop: 0,
    marginBottom: 0,
    color: theme.palette.colors.negative,
  },
}));

const RemoveDialog = ({
  open, endpoint, itemId, onResult,
}) => {
  const classes = useStyles();

  const handleRemove = async () => {
    const response = await fetch(`/api/${endpoint}/${itemId}`, { method: 'DELETE' });
    if (response.ok) {
      onResult(true);
    } else {
      onResult(false);
    }
  };

  return (
    <Snackbar
      className={classes.root}
      open={open}
      autoHideDuration={2750}
      onClose={() => onResult(false)}
      message="confirmar para eliminar"
      action={(
        <Button size="small" className={classes.button} onClick={handleRemove}>
          Eliminar
        </Button>
      )}
    />
  );
};

export default RemoveDialog;
