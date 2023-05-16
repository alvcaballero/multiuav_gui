import React, { useState,useRef ,useEffect} from 'react';
import { useDispatch, useSelector, connect } from 'react-redux';
import { map } from '../Mapview/Mapview.js';
import { useEffectAsync } from '../reactHelper';
import { dataActions, devicesActions ,missionActions,sessionActions} from '../store'; // here update device action with position of uav for update in map
const YAML = require('yaml')
const ROSLIB = require('roslib');

export const RosContext = React.createContext()

var plan_mission = "";
var mission_layers = [];
var mode_landing =0;
var mode_yaw =0;
//let uav_list = [];
let statusLog = [];

let ros = "";
const logoutCode = 4000;

export const RosControl = ({children,notification}) => {
    const devices = useSelector((state) => state.devices.items);
    const missions = useSelector((state) => state.mission);
    const dispatch = useDispatch();

    const socketRef = useRef();
  
  	const [rosState,setrosState] = useState(false);
    const [socketState,setsocketState] = useState(true);
    const [textmission,settextmission] = useState("");

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
          (data.server.rosState ==='connect')?setrosState(true):setrosState(false);
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


    const serverConecRos = async (event) => {
      //event.preventDefault();
      try {
        const response = await fetch('/api/rosConnect', {
          method: 'POST',
          body: new URLSearchParams(`rosState=${encodeURIComponent(rosState)}`),
        });
        if (response.ok) {
          let myresponse = await response.json();
          if(myresponse.state ==='connect'){
            notification('success', myresponse.msg);
            setrosState(true);
          }
          if(myresponse.state ==='error'){
            setrosState(false);
            notification('danger',myresponse.msg)
          }
          if(myresponse.state ==='disconnect'){
            notification('danger',myresponse.msg)
            setrosState(false);
          }
          console.log(myresponse)
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
        setrosState(false);
      }
    };
    const serverAddUAV = async (uav_ns, uav_type) => {
      //event.preventDefault();
      console.log(uav_type)
      console.log(uav_ns)
      try {
        const response = await fetch('/api/devices', {
          method: 'POST',
          body: JSON.stringify({uav_ns: uav_ns, uav_type: uav_type}),
          headers: {
            'Content-Type': 'application/json'
          }
        });
        if (response.ok) {
          let myresponse = await response.json();
          if(myresponse.state ==='connect'){
            notification('success', myresponse.msg);
          }
          if(myresponse.state ==='fail'){
            notification('danger',myresponse.msg)
          }
          console.log(myresponse)
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
      }
    };
    const serverRMUAV = async (uav_ns) => {
      //event.preventDefault();
      console.log(uav_ns)
      try {
        const response = await fetch('/api/disconectdevice', {
          method: 'POST',
          body: JSON.stringify({uav_ns: uav_ns}),
          headers: {
            'Content-Type': 'application/json'
          }
        });
        if (response.ok) {
          let myresponse = await response.json();
          if(myresponse.state ==='connect'){
            notification('success', myresponse.msg);
          }
          if(myresponse.state ==='fail'){
            notification('danger',myresponse.msg)
          }
          console.log(myresponse)
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
      }
    };

    const serverloadmission = async () => {
      try {
        const response = await fetch('/api/loadmission', {
          method: 'POST',
          body: JSON.stringify({mission:missions['route']}),
          headers: {
            'Content-Type': 'application/json'
          }
        });
        if (response.ok) {
          let myresponse = await response.json();
          if(myresponse.state ==='connect'){
            notification('success', myresponse.msg);
          }
          if(myresponse.state ==='fail'){
            notification('danger',myresponse.msg)
          }
          console.log(myresponse)
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
      }
    };
    const servercommandmission = async (uav_ns, uav_type) => {
      //event.preventDefault();
      console.log("command mission ")
      console.log(uav_type)
      console.log(uav_ns)
      try {
        const response = await fetch('/api/commandmission', {
          method: 'POST',
          body: JSON.stringify({uav_ns: uav_ns, uav_type: uav_type}),
          headers: {
            'Content-Type': 'application/json'
          }
        });
        if (response.ok) {
          let myresponse = await response.json();
          if(myresponse.state ==='connect'){
            notification('success', myresponse.msg);
          }
          if(myresponse.state ==='fail'){
            notification('danger',myresponse.msg)
          }
          console.log(myresponse)
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
      }
    };

    const openMision=(name_mission,text_mission)=>{
      plan_mission = YAML.parse(text_mission);
      mode_landing = plan_mission["mode_landing"];
      mode_yaw = plan_mission["mode_yaw"];
      dispatch(missionActions.clearMission())
      dispatch(missionActions.updateMission({name:name_mission,mission:plan_mission}));

      //console.log(plan_mission["uav_n"]);
      let wp_home = [0,0];

      for(let n_uav = 1; n_uav <= plan_mission["uav_n"]; n_uav++){
        if (plan_mission.hasOwnProperty("uav_"+n_uav)){
          //let wp_position =[];
          wp_home = plan_mission["uav_"+n_uav]["wp_"+0];
          //for(let n_wp = 0 ; n_wp < plan_mission["uav_"+n_uav]["wp_n"]; n_wp++){
          //  wp_position.push(plan_mission["uav_"+n_uav]["wp_"+n_wp])
          //};
          //for (var i = 0; i < uav_list.length; i++) {
          //  if(uav_list[i].name === ("uav_"+n_uav)){
          //    uav_list[i].wp_list = wp_position;//uav_list[i].wp_list =Object.values(missions.route[n_uav]['wp']);
          //  }
          //};
        }
      }
      map.easeTo({
        center: [wp_home[1],wp_home[0]],
        zoom: Math.max(map.getZoom(), 15),
        offset: [0, -1 / 2],
      });
    }

    function updateInfoCell (uav_ns, info) {
      var showData = document.getElementById(uav_ns).cells;
        showData[5].innerHTML = info;
    }

    function changeReady(uav_ns){
      var button = document.getElementById("Ready"+uav_ns);
      if(button.innerHTML === "Ready"){
          button.innerHTML = "Not Ready";
      }else{
      button.innerHTML = "Ready";
      var info = "Mission requested";
      updateInfoCell(uav_ns, info);
      }
    }

    const rosConnect = () =>{
      serverConecRos();
    }

    const getTopics2 = () => {
      var topicsClient = new ROSLIB.Service({
      ros : ros,
      name : '/rosapi/topics',
      serviceType : 'rosapi/Topics'
      });
      
      var request = new ROSLIB.ServiceRequest();
    
      return new Promise((resolve,rejects) => {
        topicsClient.callService(request, function(result) {
          resolve(result.topics);
        });
      });
    };
    

    async function connectAddUav(uav_ns,uav_type){	

      if(rosState){
        serverAddUAV(uav_ns,uav_type);
      }else{
        alert("\nRos no está conectado.\n\n Por favor conéctelo primero.")
      } 
    }

    function loadMission(){ 
      serverloadmission();
    }
  
    function commandMission(){
      servercommandmission("uav_3","px4");
    }

  return (
    <div style={{    width: '100%',height: '100%'}}>    
      <RosContext.Provider value={{rosConnect , rosState,openMision,connectAddUav,commandMission,loadMission}}>
        {children}
      </RosContext.Provider>
    </div>
  )
}
