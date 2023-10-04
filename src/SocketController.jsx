import React, { useState, useRef, useEffect } from 'react';
import { useSelector, useDispatch, connect } from 'react-redux';
import { useEffectAsync } from './reactHelper';
import alarm from './resources/alarm.mp3';
import { dataActions, devicesActions, missionActions, sessionActions } from './store'; // here update device action with position of uav for update in map
import { eventsActions } from './store/events';
import { Snackbar } from '@mui/material';

const logoutCode = 4000;

const SocketController = () => {
  const dispatch = useDispatch();

  const devices = useSelector((state) => state.devices.items);

  const socketRef = useRef();
  const [socketState, setsocketState] = useState(true);

  const [events, setEvents] = useState([]);
  const [notifications, setNotifications] = useState([]);

  const connectSocket = () => {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const socket = new WebSocket(`${protocol}//${window.location.host}/api/socket`);
    console.log(`${protocol}//${window.location.host}/api/socket`);
    //const socket = new WebSocket(`${protocol}//${window.location.host}`);
    socketRef.current = socket;
    console.log('funcion web socket');

    socket.onopen = () => {
      dispatch(sessionActions.updateSocket(true));
      console.log('funcion web socket open');
    };

    socket.onclose = async (event) => {
      console.log('funcion web socket close');
      dispatch(sessionActions.updateSocket(false));
      if (event.code !== logoutCode) {
        try {
          const devicesResponse = await fetch('/api/devices');
          if (devicesResponse.ok) {
            dispatch(devicesActions.update(await devicesResponse.json()));
          }
          const positionsResponse = await fetch('/api/positions');
          if (positionsResponse.ok) {
            dispatch(dataActions.updatePositions(await positionsResponse.json()));
          }
          if (devicesResponse.status === 401 || positionsResponse.status === 401) {
            //navigate('/login');
          }
        } catch (error) {
          // ignore errors
        }
        setTimeout(() => connectSocket(), 60000);
      }
    };

    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.devices) {
        if (data.devices.length > 0) {
          dispatch(devicesActions.update(data.devices));
        } else {
          dispatch(devicesActions.clear());
        }
      }
      if (data.positions) {
        dispatch(dataActions.updatePositions(data.positions));
      }
      if (data.camera) {
        //console.log(data.camera)
        dispatch(dataActions.updateCamera(data.camera));
      }
      if (data.server) {
        data.server.rosState === 'connect'
          ? dispatch(sessionActions.updateServer(true))
          : dispatch(sessionActions.updateServer(false));
      }
      if (data.mission) {
        console.log('data mission');
        console.log(data.mission);
        dispatch(missionActions.updateMission(data.mission));
      }
      if (data.events) {
        if (true) {
          dispatch(eventsActions.add(data.events));
        }
        setEvents(data.events);
      }
    };
  };
  useEffectAsync(async () => {
    if (socketState) {
      setsocketState(false);

      const response = await fetch('/api/devices', { method: 'GET' });
      if (response.ok) {
        dispatch(devicesActions.refresh(await response.json()));
      } else {
        throw Error(await response.text());
      }
      console.log('primera conexion --s');
      connectSocket();
      return () => {
        const socket = socketRef.current;
        if (socket) {
          socket.close(logoutCode);
        }
      };
    } else {
      return null;
    }
  }, []);

  useEffect(() => {
    setNotifications(
      events.map((event) => ({
        id: event.id,
        message: event.attributes.message,
        show: true,
      }))
    );
  }, [events, devices]);

  useEffect(() => {
    events.forEach((event) => {
      if (event.type === 'alarm') {
        new Audio(alarm).play();
      }
    });
  }, [events]);

  return (
    <>
      {notifications.map((notification) => (
        <Snackbar
          key={notification.id}
          open={notification.show}
          message={notification.message}
          autoHideDuration={snackBarDurationLongMs}
          onClose={() => setEvents(events.filter((e) => e.id !== notification.id))}
        />
      ))}
    </>
  );
};

export default connect()(SocketController);
