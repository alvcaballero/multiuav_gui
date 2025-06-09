import ReactSwipeButton from 'react-swipe-button';
import React, { useState } from 'react';
import { makeStyles } from 'tss-react/mui';


import { Card, CardContent, Typography, IconButton } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';

const useStyles = makeStyles()((theme) => ({
  card: {
    pointerEvents: 'auto',
    width: theme.dimensions.popupMaxWidth,
  },
  media: {
    height: theme.dimensions.popupImageHeight,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
  },
  content: {
    paddingTop: '20px',
    paddingBottom: '20px',
  },
  negative: {
    color: theme.palette.colors.negative,
  },
  icon: {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  },
  table: {
    '& .MuiTableCell-sizeSmall': {
      paddingLeft: 0,
      paddingRight: 0,
    },
  },
  cell: {
    borderBottom: 'none',
  },
  actions: {
    justifyContent: 'space-between',
  },
  root: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 5,
    left: '50%',
    top: '50%',
    transform: 'translate(-50%,-50%)',
  },
}));

const SwipeConfirm = ({ enable, onClose, onSucces }) => {
  const { classes } = useStyles();

  return (
    <div className={classes.root}>
      {enable && (
        <Card elevation={5} className={classes.card}>
          <div className={classes.header}>
            <Typography variant="body2" color="textSecondary">
              Confirm mission
            </Typography>
            <IconButton size="small" onClick={onClose} onTouchStart={onClose}>
              <CloseIcon fontSize="small" />
            </IconButton>
          </div>

          <CardContent className={classes.content}>
            <Typography variant="body1" color="textSecondary">
              Send all devices to command the mission load
            </Typography>
            <div style={{ marginTop: '10px', height: '50px' }}>
              <ReactSwipeButton
                text="Slide to Command Mission"
                color="#7393B3"
                text_unlocked="Commanded"
                onSuccess={onSucces}
              />
            </div>
          </CardContent>
        </Card>
      )}
    </div>
  );
};

export default SwipeConfirm;
