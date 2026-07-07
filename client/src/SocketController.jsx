import React, { useState, useRef, useCallback } from 'react';
import { useDispatch, connect } from 'react-redux';
import { useAsyncTask } from './reactHelper';
import alarm from './resources/alarm.mp3';
import {
  store,
  devicesActions,
  missionActions,
  sessionActions,
  chatActions,
  activeMissionsActions,
} from './store';
import { eventsActions } from './store/events';
import { loadMissionPlanToEditor } from './services/missionPlanLoader';
import { SnackbarProvider, enqueueSnackbar } from 'notistack';

const logoutCode = 4000;

const SocketController = () => {
  const dispatch = useDispatch();

  const socketRef = useRef();
  const [socketState, setSocketState] = useState(true);

  const handleEvents = useCallback(
    (events) => {
      dispatch(eventsActions.add(events));
      if (events.some((e) => e.type === 'error')) {
        new Audio(alarm).play();
      }
      events.forEach((event) => {
        enqueueSnackbar(event.attributes.message ? event.attributes.message : 'unknow error', {
          variant: event.type,
          autoHideDuration: 3000,
          persist: false,
        });
      });
    },
    [dispatch],
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
        } catch {
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
        // Server pushed a mission into the editor — drop any active selection.
        dispatch(activeMissionsActions.selectMission(null));
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
        dispatch(chatActions.setActiveChat(data.chatCreated.chatId));
      }
      if (data.missionProgress) {
        const { missionId } = data.missionProgress;
        const known = store.getState().activeMissions.items[missionId];
        if (!known) {
          // Mission arrived before initial fetch or was created after page load — fetch it now
          Promise.all([
            fetch(`/api/missions?id=${missionId}`).then((r) => (r.ok ? r.json() : null)),
            fetch(`/api/missions/routes?missionId=${missionId}`).then((r) =>
              r.ok ? r.json() : null,
            ),
          ]).then(([mission, routes]) => {
            if (mission)
              dispatch(
                activeMissionsActions.upsertMission(Array.isArray(mission) ? mission[0] : mission),
              );
            if (routes)
              dispatch(
                activeMissionsActions.setRoutes(
                  Array.isArray(routes) ? routes : Object.values(routes),
                ),
              );
          });
        }
        dispatch(activeMissionsActions.updateProgress(data.missionProgress));
      }
      if (data.missionCompleted) {
        dispatch(activeMissionsActions.completeMission(data.missionCompleted));
      }
    };
  };

  useAsyncTask(async () => {
    if (socketState) {
      setSocketState(false);

      const [devicesRes, missionsRes, routesRes] = await Promise.all([
        fetch('/api/devices'),
        fetch('/api/missions'),
        fetch('/api/missions/routes'),
      ]);

      if (devicesRes.ok) {
        dispatch(devicesActions.refresh(await devicesRes.json()));
      } else {
        throw Error(await devicesRes.text());
      }
      if (missionsRes.ok) {
        const missions = await missionsRes.json();
        const missionList = Array.isArray(missions) ? missions : [missions].filter(Boolean);
        dispatch(activeMissionsActions.setMissions(missionList));

        // Pre-select the most recent active mission so the tracking panel opens
        // showing something useful right away, instead of nothing selected — and
        // load its plan into the editor/map, same as clicking it manually would.
        if (missionList.length > 0) {
          const mostRecent = missionList.reduce((latest, m) =>
            new Date(m.initTime) > new Date(latest.initTime) ? m : latest,
          );
          dispatch(activeMissionsActions.selectMission(mostRecent.id));
          loadMissionPlanToEditor(mostRecent.id, dispatch);
        }
      }
      if (routesRes.ok) {
        dispatch(activeMissionsActions.setRoutes(await routesRes.json()));
      }

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

export default connect()(SocketController);
