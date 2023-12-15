// SquareMove.js
import React, { useState, useEffect } from 'react';
import makeStyles from '@mui/styles/makeStyles';
import { mapIconKey, mapIcons, frontIcons } from '../Mapview/preloadImages';
//import styled from 'styled-components';

const useStyles = makeStyles((theme) => ({
  root: {
    position: 'relative',
    width: '200px',
    height: '200px',
    left: '50px',
    backgroundColor: '#F7F9F9',
    overflow: 'hidden',
    //margin: 'auto',
  },
  box_down: {
    width: '100%',
    height: '50%',
    backgroundColor: '#5D6D7E',
    position: 'absolute',
    transition: 'bottom 0.3s ease-in-out',
    bottom: 0,
    left: '50%',
    transform: 'translate(-50%, 100%)',
    borderTop: '6px solid #1B2631 ',
  },
  box_right: {
    width: '50%',
    height: '100%',
    backgroundColor: '#5D6D7E',
    position: 'absolute',
    transition: 'right 0.3s ease-in-out',
    top: '50%',
    right: 0,
    transform: 'translate(100%, -50%)',
    borderLeft: '6px solid #1B2631 ',
  },
  box_up: {
    width: '100%',
    height: '50%',
    backgroundColor: '#5D6D7E',
    position: 'absolute',
    transition: 'top 0.3s ease-in-out',
    top: 0,
    left: '50%',
    transform: 'translate(-50%, -100%)',
    borderBottom: '6px solid #1B2631 ',
  },
  box_left: {
    width: '50%',
    height: '100%',
    backgroundColor: '#5D6D7E',
    position: 'absolute',
    transition: 'left 0.3s ease-in-out',
    top: '50%',
    left: 0,
    transform: 'translate(-100%, -50%)',
    borderRight: '6px solid #1B2631 ',
  },
  text_right: {
    position: 'absolute',
    right: '15%',
    top: '50%',
  },
  text_left: {
    position: 'absolute',
    left: '15%',
    top: '50%',
  },
  text_down: {
    position: 'absolute',
    right: '50%',
    bottom: '15%',
    transform: 'translate(50%, 50%)',
  },
  text_up: {
    position: 'absolute',
    right: '50%',
    top: '15%',
    transform: 'translate(50%, -50%)',
  },
  icon: {
    position: 'absolute',
    width: '25%',
    height: '25%',
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

const SquareMove = ({
  device,
  data = [10, 15, 11, 0, 12, 20],
  sensors = {
    down: [0, 15],
    front: [0, 15],
    left: [0, 12],
    back: [0, 12],
    right: [0, 12],
    up: [0, 20],
  },
  front_view = false,
  test = false,
}) => {
  const classes = useStyles();
  const [left, setLeft] = useState(0);
  const [right, setRight] = useState(0);
  const [up, setUp] = useState(0);
  const [down, setDown] = useState(0);

  useEffect(() => {
    if (front_view) {
      sensors.hasOwnProperty('down') ? setDown(40 - (40 * data[0]) / sensors.down[1]) : null;
      sensors.hasOwnProperty('left') ? setRight(40 - (40 * data[2]) / sensors.left[1]) : null;
      sensors.hasOwnProperty('right') ? setLeft(40 - (40 * data[4]) / sensors.right[1]) : null;
      sensors.hasOwnProperty('up') ? setUp(40 - (40 * data[5]) / sensors.up[1]) : null;
    } else {
      sensors.hasOwnProperty('front') ? setUp(40 - (40 * data[1]) / sensors.front[1]) : null;
      sensors.hasOwnProperty('left') ? setLeft(40 - (40 * data[2]) / sensors.left[1]) : null;
      sensors.hasOwnProperty('back') ? setDown(40 - (40 * data[3]) / sensors.down[1]) : null;
      sensors.hasOwnProperty('right') ? setRight(40 - (50 * data[4]) / sensors.right[1]) : null;
    }
  }, [data, device, sensors]);

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
        <div className={classes.box_left} style={{ left: `${left}%` }}></div>
        <div className={classes.box_right} style={{ right: `${right}%` }}></div>
        <div className={classes.box_up} style={{ top: `${up}%` }}></div>
        <div className={classes.box_down} style={{ bottom: `${down}%` }}></div>

        {!front_view && (
          <>
            {left < 50 && <div className={classes.text_left}> {`${data[2]}m`}</div>}
            {right < 50 && <div className={classes.text_right}> {`${data[4]}m`}</div>}
            {up < 50 && <div className={classes.text_up}> {`${data[1]}m`}</div>}
            {down < 50 && <div className={classes.text_down}> {`${data[3]}m`}</div>}
            <img className={classes.icon} src={mapIcons[mapIconKey('dji_M210_melodic')]} alt='' />
          </>
        )}
        {front_view && (
          <>
            {left < 50 && <div className={classes.text_left}> {`${data[4]}m`}</div>}
            {right < 50 && <div className={classes.text_right}> {`${data[2]}m`}</div>}
            {up < 50 && <div className={classes.text_up}> {`${data[5]}m`}</div>}
            {down < 50 && <div className={classes.text_down}> {`${data[0]}m`}</div>}
            <img className={classes.icon} src={frontIcons[mapIconKey('dji_M210_melodic')]} alt='' />
          </>
        )}
      </div>

      {test && (
        <div>
          <button onClick={handleMoveLeft}>Move Left</button>
          <button onClick={handleMoveRight}>Move Right</button>
        </div>
      )}
    </div>
  );
};

export default SquareMove;
