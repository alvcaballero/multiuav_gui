// SquareMove.js
import React, { useState } from 'react';
import makeStyles from '@mui/styles/makeStyles';
import { mapIconKey, mapIcons } from '../Mapview/preloadImages';
//import styled from 'styled-components';

const useStyles = makeStyles((theme) => ({
  root: {
    position: 'relative',
    width: '500px',
    height: '500px',
    left: '50px',
    backgroundColor: 'grey',
    //overflow: 'hidden',
    //margin: 'auto',
  },
  box_down: {
    width: '100%',
    height: '50px',
    backgroundColor: 'blue',
    position: 'absolute',
    transition: 'bottom 0.3s ease-in-out',
    bottom: 0,
    left: '50%',
    transform: 'translate(-50%, -50%)',
  },
  box_right: {
    width: '50px',
    height: '100%',
    backgroundColor: 'blue',
    position: 'absolute',
    transition: 'right 0.3s ease-in-out',
    top: '50%',
    right: 0,
    transform: 'translate(+50%, -50%)',
  },
  box_up: {
    width: '100%',
    height: '50px',
    backgroundColor: 'blue',
    position: 'absolute',
    transition: 'top 0.3s ease-in-out',
    top: 0,
    left: '50%',
    transform: 'translate(-50%, -50%)',
  },
  box_left: {
    width: '50px',
    height: '100%',
    backgroundColor: 'blue',
    position: 'absolute',
    transition: 'left 0.3s ease-in-out',
    top: '50%',
    left: 0,
    transform: 'translate(-50%, -50%)',
  },
  icon: {
    position: 'absolute',
    width: '100px',
    height: '100px',
    top: '50%',
    left: '50%',
    transform: 'translate(-50%, -50%)',
  },
  centerContent: {
    position: 'absolute',
    top: '50%',
    left: '50%',
    transform: 'translate(-50%, -50%)',
  },
}));

const SquareMove = () => {
  const classes = useStyles();
  const [left, setLeft] = useState(0);
  const [right, setRight] = useState(0);
  const [up, setUp] = useState(0);
  const [mydown, setDown] = useState(0);

  const handleMoveLeft = () => {
    setLeft((prevLeft) => prevLeft - 10);
    setRight((prevLeft) => prevLeft - 10);
    setUp((prevLeft) => prevLeft - 10);
    setDown((prevLeft) => prevLeft - 10);
  };

  const handleMoveRight = () => {
    setLeft((prevLeft) => prevLeft + 10);
    setRight((prevLeft) => prevLeft + 10);
    setUp((prevLeft) => prevLeft + 10);
    setDown((prevLeft) => prevLeft + 10);
  };

  return (
    <div>
      <div className={classes.root}>
        <div className={classes.box_left} style={{ left: `${left}px` }}></div>
        <div className={classes.box_right} style={{ right: `${right}px` }}></div>
        <div className={classes.box_up} style={{ top: `${up}px` }}></div>
        <div className={classes.box_down} style={{ bottom: `${mydown}px` }}></div>

        <img className={classes.icon} src={mapIcons[mapIconKey('dji_M210_melodic')]} alt='' />
      </div>

      <button onClick={handleMoveLeft}>Move Left</button>
      <button onClick={handleMoveRight}>Move Right</button>
    </div>
  );
};

export default SquareMove;
