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
let uav_list = [];
let statusLog = [];

let ros = "";
const logoutCode = 4000;

export const RosControl = ({children,notification}) => {
    const devices = useSelector((state) => state.devices.items);
    const missions = useSelector((state) => state.mission);
    const dispatch = useDispatch();

    const socketRef = useRef();
  
  	const [rosState,setrosState] = useState(false);
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
        //console.log("funcion web socket message")
        //console.log(event)
        const data = JSON.parse(event.data);
        //console.log(data.positions)
        if (data.devices ) {
          dispatch(devicesActions.update(Object.values(data.devices)));
        }
        if (data.positions) {
          console.log(data.positions)
          dispatch(dataActions.updatePositions(Object.values(data.positions)));
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

    const serverloadmission = async (uav_ns, uav_type) => {
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
    const servercommandmission = async (uav_ns, uav_type) => {
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

    const openMision=(name_mission,text_mission)=>{
      plan_mission = YAML.parse(text_mission);
      mode_landing = plan_mission["mode_landing"];
      mode_yaw = plan_mission["mode_yaw"];
      dispatch(missionActions.updateMission({name:name_mission,mission:plan_mission}));

      //console.log(plan_mission["uav_n"]);
      let wp_home = [0,0];

      for(let n_uav = 1; n_uav <= plan_mission["uav_n"]; n_uav++){
        if (plan_mission.hasOwnProperty("uav_"+n_uav)){
          let wp_position =[];
          wp_home = plan_mission["uav_"+n_uav]["wp_"+0];
          for(let n_wp = 0 ; n_wp < plan_mission["uav_"+n_uav]["wp_n"]; n_wp++){
            wp_position.push(plan_mission["uav_"+n_uav]["wp_"+n_wp])
          };
          for (var i = 0; i < uav_list.length; i++) {
            if(uav_list[i].name === ("uav_"+n_uav)){
              uav_list[i].wp_list = wp_position;//uav_list[i].wp_list =Object.values(missions.route[n_uav]['wp']);
            }
          };
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

    const removeMarker = () =>{
      if (mission_layers!==null) {
        for (var i = mission_layers.length - 1; i >= 0; i--) {
          mission_layers[i].remove();
        }
      }
    }


    const rosConnect = () =>{
      serverConecRos();
      //if(!rosState){
      //  ros = new ROSLIB.Ros({url : 'ws://localhost:9090'});
      //  ros.on('connection', function() {
      //    console.log('Connected to websocket server.');
      //    setrosState(true);
      //  });
      //  ros.on('error', function(error) {
      //    console.log('Error connecting to websocket server: ', error);
      //    notification('danger','no se puede conectar');
      //  });
      //  ros.on('close', function() {
      //    console.log('Connection to websocket server closed.');
      //  });
      //}else{
      //  for (var i = 0; i < uav_list.length; i++) {
      //    uav_list[i].listener.unsubscribe();   
      //    uav_list[i].pose = [0, 0];
      //  }
      //  uav_list = [];
      //  ros.close();
      //  setrosState(false);
      //}
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
      
      //var text = document.getElementById("rosConnect");
  
      if(rosState){
        serverAddUAV(uav_ns,uav_type);
// 
//        const topiclist = await getTopics2();
//        console.log("despues del await")
//
//        alert("Conectando Uav");
//
//        let marker_n = uav_list.length;
//        let iconSelect = '../assets/css/images/' + uav_type + '-marker' + marker_n.toString() + '.png';
//        //uav_icon = L.icon({iconUrl: iconSelect, iconSize: [24, 24],iconAnchor: [12, 12]});
//        let find_device = false;
//        let repeat_device = false;
//        topiclist.forEach(element =>{
//          find_device = element.includes(uav_ns) || find_device;
//          //console.log(element);
//        });
//        uav_list.forEach(element =>{
//          if( element.name === uav_ns){
//            repeat_device =true;
//          }
//        });
//        if( find_device === false){ 
//          alert("Dispositivo no encontrado"+uav_ns);
//          return null
//        }
//        if( repeat_device === true){ 
//          alert("Dispositivo ya se encuentra anadido"+uav_ns);
//          return null
//        }
//
//        let cur_uav_idx = String(Object.values(devices).length)
//
//        dispatch(devicesActions.update({id:cur_uav_idx,name:uav_ns,category:uav_type,status:'online'}));
//
//        
//        let uavAdded = { name : uav_ns, type : uav_type, watch_bound : true, wp_list : [] , listener : "", listener_hdg : "", listener_alt : "", listener_vel : "", listener_bat : "",listener_cam : "",bag : false};
//        uav_list.push(uavAdded);
//
//        // Subscribing
//        // DJI 
//        if(uav_type === "dji"){
//  
//          uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/dji_osdk_ros/rtk_position',
//            messageType : 'sensor_msgs/NavSatFix'
//          });
//          uav_list[cur_uav_idx].listenerov = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/dji_osdk_ros/vo_position',
//            messageType : 'dji_osdk_ros/VOPosition'
//          });
//          uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/dji_osdk_ros/rtk_yaw',
//            messageType : 'std_msgs/Int16'
//          });
//  
//          uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/dji_osdk_ros/velocity',
//            messageType : 'geometry_msgs/Vector3Stamped'
//          });
//  
//          uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/dji_osdk_ros/battery_state',
//            messageType : 'sensor_msgs/BatteryState'
//          });
//  
//          uav_list[cur_uav_idx].listener_cam = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/video_stream_compress',
//            messageType : 'sensor_msgs/CompressedImage'
//          });
//          uav_list[cur_uav_idx].camera_stream = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/video_stream',
//            messageType : 'sensor_msgs/Image'
//          });
//          
//  
//          uav_list[cur_uav_idx].listener.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;
//            dispatch(dataActions.updatePosition({id : msg.header.seq,deviceId:id_uav,  latitude:msg.latitude,longitude:msg.longitude, altitude:msg.altitude,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));
//            
//          });
//          uav_list[cur_uav_idx].listenerov.subscribe(function(msg) {
//            //let id_uav = cur_uav_idx;
//            //dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.x,longitude:msg.y,altitude:msg.z,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));          
//          });
//
//          uav_list[cur_uav_idx].listener_hdg.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;
//            dispatch(dataActions.updatePosition({deviceId:id_uav,course:msg.data+90}));//uav_list[cur_uav_idx].marker.setRotationAngle(message.data+90);
//          });				
//          
//          uav_list[cur_uav_idx].listener_vel.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;// var showData = document.getElementById(uav_ns).cells;
//           dispatch(dataActions.updatePosition({deviceId:id_uav,speed:Math.sqrt(Math.sqrt(Math.pow(msg.vector.x,2) + Math.pow(msg.vector.y,2)).toFixed(2))}));// showData[2].innerHTML = Math.sqrt(Math.pow(message.vector.x,2) + Math.pow(message.vector.y,2)).toFixed(2);
//          });				
//          
//          uav_list[cur_uav_idx].listener_bat.subscribe(function(msg) {
//            let id_uav = cur_uav_idx//var showData = document.getElementById(uav_ns).cells;
//            dispatch(dataActions.updatePosition({deviceId:id_uav,batteryLevel:msg.percentage}));// showData[3].innerHTML = message.percentage + "%";
//          });
//  
//          uav_list[cur_uav_idx].listener_cam.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;
//            console.log("camradji")
//            dispatch(dataActions.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data}));//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
//            //document.getElementById('my_image').src = "data:image/bgr8;base64," + message.data;
//          });
//          
//        }else if(uav_type === "px4"){
//  
//          uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/global_position/global',
//            messageType : 'sensor_msgs/NavSatFix'
//          });
//
//          uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/global_position/compass_hdg',
//            messageType : 'std_msgs/Float64'
//          });
//  
//          uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/local_position/velocity_local',
//            messageType : 'geometry_msgs/TwistStamped'
//          });
//  
//          uav_list[cur_uav_idx].listener_alt = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/altitude',
//            messageType : 'mavros_msgs/Altitude'
//          });
//  
//          uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/battery',
//            messageType : 'sensor_msgs/BatteryState'
//          });
//          uav_list[cur_uav_idx].listener_cam = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/video_stream_compress',
//            messageType : 'sensor_msgs/CompressedImage'
//          });
//  
//          uav_list[cur_uav_idx].listener.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;
//            dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.latitude,longitude:msg.longitude,altitude:msg.altitude,deviceTime:"2023-03-09T22:12:44.000+00:00"})); 
//          });
//  
//          uav_list[cur_uav_idx].listener_hdg.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;
//            dispatch(dataActions.updatePosition({deviceId:id_uav,course:msg.data}));//uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
//          });
//  
//          uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
//            //var showData = document.getElementById(uav_ns).cells;
//            //showData[1].innerHTML = (message.relative).toFixed(2);   
//          });
//  
//          uav_list[cur_uav_idx].listener_vel.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;//var showData = document.getElementById(uav_ns).cells;
//            dispatch(dataActions.updatePosition({deviceId:id_uav,speed:Math.sqrt(Math.pow(msg.twist.linear.x,2) + Math.pow(msg.twist.linear.y,2)).toFixed(2)}));// showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
//          });
//  
//          uav_list[cur_uav_idx].listener_bat.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;//var showData = document.getElementById(uav_ns).cells;
//            dispatch(dataActions.updatePosition({deviceId:id_uav,batteryLevel:(msg.percentage*100).toFixed(0)}));//  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
//          });
//          uav_list[cur_uav_idx].listener_cam.subscribe(function(msg) {
//            let id_uav = cur_uav_idx;
//            dispatch(dataActions.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data}));//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
//            //document.getElementById('my_image').src = "data:image/bgr8;base64," + message.data;
//          });
//
//
//        } else {
//          //EXT (FUVEX)
//          uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/global_position/global',
//            messageType : 'sensor_msgs/NavSatFix'
//          });
//  
//          uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/global_position/compass_hdg',
//            messageType : 'std_msgs/Float64'
//          });
//  
//          uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/local_position/velocity_local',
//            messageType : 'geometry_msgs/TwistStamped'
//          });
//  
//          uav_list[cur_uav_idx].listener_alt = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/altitude',
//            messageType : 'mavros_msgs/Altitude'
//          });
//  
//          uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
//            ros : ros,
//            name : uav_ns+'/mavros/battery',
//            messageType : 'sensor_msgs/BatteryState'
//          });
//  
//          uav_list[cur_uav_idx].listener.subscribe(function(message) {
//            uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
//            uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);
//          });
//  
//          uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
//            uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
//          });
//  
//          uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
//            var showData = document.getElementById(uav_ns).cells;
//              showData[1].innerHTML = (message.relative).toFixed(2);										
//          });
//  
//          uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
//            var showData = document.getElementById(uav_ns).cells;
//              showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
//          });
//  
//          uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
//            var showData = document.getElementById(uav_ns).cells;
//              showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
//          });
//
//
//        }
//  
//        console.log("\nLa Lista de uav's es: ");
//        uav_list.forEach(function(uav, indice, array) {
//          console.log(uav, indice);
//        })
//        notification('success', uavAdded.name + " added. Type: "+ uavAdded.type);
      }else{
        alert("\nRos no está conectado.\n\n Por favor conéctelo primero.")
      } 
    }

    function loadMission(){ 
      // console.log(control.getSelection())
      // let path_wp = control.getSelection()
      let cur_roster = []
      let cur_ns = ""
      uav_list.forEach(function prepare_wp(item,idx,arr){
        cur_ns = item.name
        cur_roster.push(cur_ns);
        if (item.type !== "ext"){
          let wp_command = [];
          let yaw_pos =[];
            item.wp_list.forEach(
              function prepare_wp(item,idx,arr){
                let pos = new ROSLIB.Message({
                  latitude: item[0], 
                  longitude: item[1],
                  altitude: item[2]  //Creo que aqui si añadimos otro campo, cuando le des arriba a load mission se subiria bien añadiendo yaw, por ej. Load missiontouav creo que es para subirle la mision de forma individual a cada uno, desde la tabla
                });
                
              yaw_pos.push(item[3])
              wp_command.push(pos);
            });
            
            let yaw_pos_msg = new ROSLIB.Message({
              data: yaw_pos
            });
  
          let missionClient;
          if(item.type === "dji"){
            missionClient = new ROSLIB.Service({
              ros : ros,
              name : cur_ns + '/dji_control/configure_mission',
              serviceType : 'aerialcore_common/ConfigMission'
            });
          }else{
            missionClient = new ROSLIB.Service({
              ros : ros,
              name : cur_ns + '/mission/new',
              serviceType : 'aerialcore_common/ConfigMission'
            });
          }
  
          var request = new ROSLIB.ServiceRequest({
            type : "waypoint",
            waypoint: wp_command,
            radius : 0,
            maxVel:	3,
            idleVel: 1.8,
            yaw: yaw_pos_msg,
            yawMode: mode_yaw,
            traceMode: 0,
            finishAction: mode_landing
          });
          console.log(request)
  
          missionClient.callService(request, function(result) {
            console.log('load mission'+ missionClient.name+': '+result.success);
              if(result.success){
                notification('success',"Load mission to:" + cur_roster + " ok");
              } else{
                notification('danger',"Load mission to:" + cur_roster + " fail");
              }
          }, function(result) {
            console.log('Error:'
            + result);
            alert('Error:'
            + result);
          });
          console.log('loading mision to'+cur_ns);			
        } else{
          // if (item.type == ext)
          // Si mandamos la mision individual para cada uav, hay que hacerlo por este camino, y descomentar la linea siguiente.
          //loadMissionToExt(cur_ns);
          // var info = "Loading Mission";
          // updateInfoCell(cur_ns, info);
        }
      
      })
      // La linea siguiente tiene en cuenta que se mandan las misiones de todos los uavs EXT a la vez. Si no fuese así, habría que comentar la linea siguiente.
      // loadMissionToExt(cur_ns);
      let flyButton = document.getElementById("commandMission");
      if(flyButton.style.cssText === "visibility: none;"){
        flyButton.style.cssText = "visibility: hidden;"
      }else{
        flyButton.style.cssText = "visibility: none;"
      }
      notification('success',"Loadding mission to: " + cur_roster);
    }
  

    function commandMission(){
      //var r = confirm("Comand mission?");
      var r =true;
      if (r === true) {
        let cur_roster = []
        uav_list.forEach(function prepare_wp(uav,idx,arr){
          cur_roster.push(uav.name);
          let missionClient;
          if (uav.type !== "ext"){
            if(uav.type === "dji"){
              missionClient = new ROSLIB.Service({
                ros : ros,
                name : uav.name+'/dji_control/start_mission',
                serviceType : 'std_srvs/SetBool'
              });
  
            }else{
              missionClient = new ROSLIB.Service({
                ros : ros,
                name : uav.name+'/mission/start_stop',
                serviceType : 'std_srvs/SetBool'
              });
            }
          
            let request = new ROSLIB.ServiceRequest({data: true});
  
            missionClient.callService(request, function(result) {
              console.log(result.message);
              if(result.success){
                notification('success',"Start mission:" + cur_roster + " ok");
              } else{
                notification('danger',"Start mission:" + cur_roster + " FAIL!");
              }
            });
          } else {
            console.log(uav.name);
            //commandMissionToEXT(uav.name);
          }
        });
        notification('success',"Commanding mission to: " + cur_roster);
      } else {
        console.log("Mission canceled")
      }
    }

  return (
    <div style={{    width: '100%',height: '100%'}}>    
      <RosContext.Provider value={{rosConnect , rosState,openMision,connectAddUav,commandMission,loadMission}}>
        {children}
      </RosContext.Provider>
    </div>
  )
}
