import React , { useState,useRef,useEffect} from 'react'
import { useDispatch ,connect } from 'react-redux';
import { useEffectAsync } from './reactHelper';
import { dataActions, devicesActions ,sessionActions} from './store'; // here update device action with position of uav for update in map

const logoutCode = 4000;

const SocketController = () => {
    const dispatch = useDispatch();
    
    const socketRef = useRef();
    const [socketState,setsocketState] = useState(true);

    const connectSocket = () => {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const socket = new WebSocket(`${protocol}//${window.location.host}/api/socket`);
        //const socket = new WebSocket(`${protocol}//${window.location.host}`);
        socketRef.current = socket;
        console.log("funcion web socket")
    
        socket.onopen = () => {
          dispatch(sessionActions.updateSocket(true));
          console.log("funcion web socket open")
        };
    
        socket.onclose = async (event) => {
          console.log("funcion web socket close")
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
          if (data.devices ) {
            if(Object.values(data.devices).length>0){
              dispatch(devicesActions.update(Object.values(data.devices)));
            }else{
              dispatch(devicesActions.clear());
            }
          }
          if (data.positions) {
            dispatch(dataActions.updatePositions(Object.values(data.positions)));
          }
          if (data.server){
            //(data.server.rosState ==='connect')?setrosState(true):setrosState(false);
          }
          //if (data.events) {
          //  if (!features.disableEvents) {
          //    dispatch(eventsActions.add(data.events));
          //  }
          //  setEvents(data.events);
          //}
        };
      };
      useEffectAsync(async () => {
        if(socketState){
          setsocketState(false);
        
        const response = await fetch('/api/devices',{method: 'GET'});
        if (response.ok) {
          //dispatch(devicesActions.refresh(await response.json()));
        } else {
          //throw Error(await response.text());
        }
        console.log("primera conexion --s")
        connectSocket();
        return () => {
          const socket = socketRef.current;
          if (socket) {
            socket.close(logoutCode);
          }
        };
      }
      else{
        return null;
      }
      }, []);


  return (
    <>
    </>
  );
};

export default connect()(SocketController);
