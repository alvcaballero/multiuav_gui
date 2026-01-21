import React, { useState, useRef, useEffect, useCallback } from 'react';
import { useSelector, useDispatch, connect } from 'react-redux';
import { useEffectAsync } from './reactHelper';
import alarm from './resources/alarm.mp3';
import { devicesActions, missionActions, sessionActions, chatActions } from './store'; // here update device action with position of uav for update in map
import { eventsActions } from './store/events';
import { Snackbar } from '@mui/material';
import { SnackbarProvider, enqueueSnackbar, useSnackbar } from 'notistack';
import store from './store';

const logoutCode = 4000;
const snackBarDurationLongMs = 1000;

const SocketController = () => {
  const dispatch = useDispatch();

  const socketRef = useRef();
  const [socketState, setsocketState] = useState(true);

  const [notifications, setNotifications] = useState([]);

  const handleEvents = useCallback(
    (events) => {
      dispatch(eventsActions.add(events));
      if (events.some((e) => e.type === 'error')) {
        new Audio(alarm).play();
      }
      setNotifications(
        events.map((event) => ({
          id: event.id,
          type: event.type,
          message: event.attributes.message,
          show: true,
        }))
      );
    },
    [dispatch, setNotifications]
  );

  const connectSocket = () => {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const socket = new WebSocket(`${protocol}//${window.location.host}/api/socket`);
    console.log(`${protocol}//${window.location.host}/api/socket`);
    //const socket = new WebSocket(`${protocol}//${window.location.host}`);
    socketRef.current = socket;
    window.websocket = socket; // Store socket reference globally for sendChatMessage
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
            dispatch(sessionActions.updatePositions(await positionsResponse.json()));
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
        dispatch(devicesActions.update(data.devices));
      }
      if (data.positions) {
        dispatch(sessionActions.updatePositions(data.positions));
      }
      if (data.camera) {
        dispatch(sessionActions.updateCamera(data.camera));
      }
      if (data.server) {
        data.server.rosState === 'connect'
          ? dispatch(sessionActions.updateServerROS(true))
          : dispatch(sessionActions.updateServerROS(false));
      }
      if (data.mission) {
        console.log(data.mission);
        dispatch(missionActions.updateMission(data.mission));
      }
      if (data.events) {
        handleEvents(data.events);
      }
      if (data.markers) {
        dispatch(sessionActions.updateMarker(data.markers));
      }
      if (data.planning) {
        dispatch(sessionActions.updatePlanning(data.planning));
      }
      if (data.chat) {
        dispatch(chatActions.addMessage(data.chat));
      }
      if (data.chatCreated) {
        // Server created a new chat, update the active chat ID
        dispatch(chatActions.setActiveChat(data.chatCreated.chatId));
      }
    };
  };

  useEffectAsync(async () => {
    if (socketState) {
      setsocketState(false);
      const response = await fetch('/api/devices');
      if (response.ok) {
        dispatch(devicesActions.refresh(await response.json()));
      } else {
        throw Error(await response.text());
      }
      console.log('Socket first connection');
      connectSocket();
      return () => {
        const socket = socketRef.current;
        if (socket) {
          socket.close(logoutCode);
        }
      };
    }
    return null;
  }, []);

  useEffect(() => {
    console.log('notifications');
    console.log(notifications);
    for (let i = 0; i < notifications.length; i += 1) {
      enqueueSnackbar(notifications[i].message ? notifications[i].message : 'unknow error', {
        variant: notifications[i].type,
        autoHideDuration: 3000,
        persist: false,
      });
    }
  }, [notifications]);

  return (
    <>
      <SnackbarProvider
        preventDuplicate
        maxSnack={6}
        autoHideDuration={5000}
        anchorOrigin={{
          vertical: 'bottom',
          horizontal: 'right',
        }}
      />
    </>
  );
};

// Helper function to send chat messages via WebSocket
export const sendChatMessage = (chatId, message) => {
  // Get the socket from the store or window
  const socket = window.websocket;

  if (!socket || socket.readyState !== WebSocket.OPEN) {
    console.error('WebSocket not connected');
    throw new Error('WebSocket not connected');
  }

  const payload = {
    type: 'chat:user_message',
    payload: {
      chatId: chatId,
      message: message,
      timestamp: new Date().toISOString(),
    }
  };

  console.log('Sending chat message via WebSocket:', payload);
  socket.send(JSON.stringify(payload));
};

export default connect()(SocketController);
