import React, { useState } from 'react';
import { useDispatch, useSelector, connect } from 'react-redux';
import maplibregl from 'maplibre-gl';
import { map } from '../Mapview/Mapview.js';
import { dataActions, devicesActions } from '../store'; // here update device action with position of uav for update in map
const YAML = require('yaml')
const ROSLIB = require('roslib');

export const RosContext = React.createContext()


var uav_pose = [0,0];
var plan_mission = "";
var mission_layers = [];
var mode_landing =0;
let uav_list = [];
let statusLog = [];



let ros = "";

export const RosControl = ({children}) => {
    const devices = useSelector((state) => state.devices.items);
    const dispatch = useDispatch();
  
  	const [rosState,setrosState] = useState(false);
    const [textmission,settextmission] = useState("");

    const openMision=(text_mission)=>{
      let plan_mission = YAML.parse(text_mission);
      console.log(plan_mission["uav_n"]);
      removeMarker();
      
      for(let n_uav = 1; n_uav <= plan_mission["uav_n"]; n_uav++){
        let latlngs = [];
        if (map.getLayer('route_'+n_uav)) {//if (map.getSource(id)) 
          map.removeLayer('route_'+n_uav);
          map.removeSource('route_'+n_uav);
        }
        //mission_layers.push(L.layerGroup([]));
        for(let n_wp = 0 ; n_wp < plan_mission["uav_"+n_uav]["wp_n"]; n_wp++){
          latlngs.push([plan_mission["uav_"+n_uav]["wp_"+n_wp][1],plan_mission["uav_"+n_uav]["wp_"+n_wp][0]])
          const oneMarker = new maplibregl.Marker().setLngLat([plan_mission["uav_"+n_uav]["wp_"+n_wp][1],plan_mission["uav_"+n_uav]["wp_"+n_wp][0]])
            .setPopup(new maplibregl.Popup().setText('pup up')).addTo(map);
            mission_layers.push(oneMarker)

         // mission_layers[n_uav-1].addLayer(L.marker(plan_mission["uav_"+n_uav]["wp_"+n_wp],{icon: wp_blue_icon})
          //  .bindPopup('<b>wp_'+n_wp+' (uav_'+n_uav+')</b> <br>lat: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][0]+'<br>long: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][1]+'<br>alt: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][2]))
        };
        map.easeTo({
          center: latlngs[0],
          zoom: Math.max(map.getZoom(), 10),
          offset: [0, -1 / 2],
        });
        map.addSource('route_'+n_uav, {
          'type': 'geojson',
          'data': {
            'type': 'Feature',
            'properties': {},
            'geometry': {
            'type': 'LineString',
            'coordinates':latlngs
                        }
                  }
          });
          map.addLayer({
          'id': 'route'+n_uav,
          'type': 'line',
          'source': 'route_'+n_uav,
          'layout': {
          'line-join': 'round',
          'line-cap': 'round'
          },
          'paint': {
          'line-color': '#888',
          'line-width': 8
          }
          });
        //let polygon = L.polyline(latlngs,{color: rand_color()}).bindPopup("uav_"+n_uav);
        //mission_layers[n_uav-1].addLayer(polygon).addTo(map);
        //map.fitBounds(polygon.getBounds());
        //let uav_idx = uav_list.findIndex(element => element.name == "uav_"+n_uav)
        // if findIndex fail uav_idx = -1 
        //if(uav_idx >= 0 ){
        //  uav_list[uav_idx].wp_list = latlngs
        //}
        //showFlyButton()
      }

    }



    function updateStatusWindow (text) {
      var statusLenght;
      var status = document.getElementById("statusWindow");
      statusLenght = statusLog.push("\n"+text);
      if (statusLenght > 7){
        statusLog.shift();
      }
      status.innerHTML = statusLog;
    }

    function updateInfoCell (uav_ns, info) {
      var showData = document.getElementById(uav_ns).cells;
        showData[5].innerHTML = info;
    }

    function changeReady(uav_ns){
      var button = document.getElementById("Ready"+uav_ns);
      if(button.innerHTML == "Ready"){
        //var r = confirm("Change to Not Ready");
        //if (r == true) {
          button.innerHTML = "Not Ready";
        //  } else{
        //  console.log("UAV still Ready");
        //}
      }else{
      button.innerHTML = "Ready";
      // fila.cell5.buttonReady.innerHTML = "Ready";
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
      //var text = document.getElementById("rosConnect");
      if(!rosState){
        ros = new ROSLIB.Ros({url : 'ws://localhost:9090'});
        ros.on('connection', function() {
          console.log('Connected to websocket server.');
          setrosState(true);
        });
        ros.on('error', function(error) {
          console.log('Error connecting to websocket server: ', error);
        });
        ros.on('close', function() {
          console.log('Connection to websocket server closed.');
        });
      }else{
        for (var i = 0; i < uav_list.length; i++) {
          uav_list[i].listener.unsubscribe();
          uav_list[i].marker.setOpacity(0);				
          uav_list[i].pose = [0, 0];
        }
        uav_list = [];
        ros.close();
        setrosState(false);
      }
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
 
        const topiclist = await getTopics2();
        console.log("despues del await")

        alert("Conectando Uav");

        let marker_n = uav_list.length;
        let iconSelect = '../assets/css/images/' + uav_type + '-marker' + marker_n.toString() + '.png';
        //uav_icon = L.icon({iconUrl: iconSelect, iconSize: [24, 24],iconAnchor: [12, 12]});
        let find_device = false;
        let repeat_device = false;
        topiclist.forEach(element =>{
          find_device = element.includes(uav_ns) || find_device;
          //console.log(element);
        });
        uav_list.forEach(element =>{
          if( element.name == uav_ns){
            repeat_device =true;
          }
        });
        if( find_device == false){ 
          alert("Dispositivo no encontrado"+uav_ns);
          return null
        }
        if( repeat_device == true){ 
          alert("Dispositivo ya se encuentra anadido"+uav_ns);
          return null
        }
        let cur_uav_idx = String(Object.values(devices).length+1)

        dispatch(devicesActions.update({id:cur_uav_idx,name:uav_ns,category:uav_type,status:'online'}));

  

        let uav_marker = new maplibregl.Marker().setLngLat(uav_pose).setPopup(new maplibregl.Popup().setText('pup up')).addTo(map);
        
        let uavAdded = { name : uav_ns, type : uav_type, pose : [] , marker : uav_marker, watch_bound : true, wp_list : [] , listener : "", listener_hdg : "", listener_alt : "", listener_vel : "", listener_bat : "",bag : false};
        uav_list.push(uavAdded);
        uav_list.push(uavAdded);
        uav_list.push(uavAdded);
        //var cur_uav_idx = uav_list.length-1;
        console.log("valor de cur uav_idx");
        console.log(cur_uav_idx)

        var buttonReady = document.createElement("button");
        buttonReady.setAttribute("class", "btn btn-default icon-front");
        buttonReady.setAttribute("id", "Ready"+uav_ns);
        buttonReady.innerHTML = "Not Ready";
        buttonReady.style.marginLeft = 0;
        buttonReady.addEventListener("click", function () {
          changeReady(uav_ns);				
          });			
        //cell5.appendChild(buttonReady);
  
        // Add some text to the new cells:
              
        //cell1.innerHTML = uavAdded.name;
        //cell1.float = "right";
        // Add button & image after uav name
        var buttonMarker = document.createElement("button");
        //buttonMarker.onclick = function () {
        //  GoToUavPosition(map, cur_uav_idx);				
        //  };
        buttonMarker.setAttribute("class", "uav-buttonMarker");
        //cell1.appendChild(buttonMarker);
              
        // Subscribing
        // DJI 
        if(uav_type == "dji"){
  
          var buttonLoad = document.createElement("button");
          buttonLoad.setAttribute("class", "btn btn-default icon-front icon icon-upload");
          // buttonLoad.style.marginLeft = '25%';
          //buttonLoad.addEventListener("click", function () {
          //  loadMissionToUAV(uav_ns);				
          //  });			
          //cell5.appendChild(buttonLoad);
          var buttonFlyToUav = document.createElement("button");
          buttonFlyToUav.setAttribute("class", "btn btn-default icon-front");
          buttonFlyToUav.innerHTML = "Fly!";			
          //buttonFlyToUav.addEventListener("click", function () {
          //commandMissionToUAV(uav_ns);				
          //    });			
          //cell5.appendChild(buttonFlyToUav);
  
          uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/dji_osdk_ros/rtk_position',
            messageType : 'sensor_msgs/NavSatFix'
          });
          uav_list[cur_uav_idx].listenerov = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/dji_osdk_ros/vo_position',
            messageType : 'dji_osdk_ros/VOPosition'
          });
          uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/dji_osdk_ros/rtk_yaw',
            messageType : 'std_msgs/Int16'
          });
  
          uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/dji_osdk_ros/velocity',
            messageType : 'geometry_msgs/Vector3Stamped'
          });
  
          uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/dji_osdk_ros/battery_state',
            messageType : 'sensor_msgs/BatteryState'
          });
  
          uav_list[cur_uav_idx].camera_stream_comprese = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/video_stream_compress',
            messageType : 'sensor_msgs/CompressedImage'
          });
          uav_list[cur_uav_idx].camera_stream = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/video_stream',
            messageType : 'sensor_msgs/Image'
          });
          
  
          uav_list[cur_uav_idx].listener.subscribe(function(msg) {
            let id_uav = cur_uav_idx;
            dispatch(dataActions.updatePosition({id : msg.header.seq,deviceId:id_uav,  latitude:msg.latitude,longitude:msg.longitude, altitude:msg.altitude,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));


            // console.log('Received message on ' + listen_gps_pos.name + ': ' + message.latitude);
            //uav_list[cur_uav_idx].pose = [ message.longitude ,message.latitude];
            //uav_list[cur_uav_idx].marker.setLngLat(uav_list[cur_uav_idx].pose);
            //uav_list[cur_uav_idx].marker.addTo(map);
            
          });
          uav_list[cur_uav_idx].listenerov.subscribe(function(msg) {
            //let id_uav = cur_uav_idx;
            //dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.x,longitude:msg.y,altitude:msg.z,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));          
          });

          uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
            //uav_list[cur_uav_idx].marker.setRotationAngle(message.data+90);
            var markerRotation = message.data+90;
            buttonMarker.style.transform = 'rotate(' + markerRotation.toString() + 'deg)';
          });				
          
          uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
           // var showData = document.getElementById(uav_ns).cells;
             // showData[2].innerHTML = Math.sqrt(Math.pow(message.vector.x,2) + Math.pow(message.vector.y,2)).toFixed(2);
          });				
          
          uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
            //var showData = document.getElementById(uav_ns).cells;
            // showData[3].innerHTML = message.percentage + "%";
          });
  
          uav_list[cur_uav_idx].camera_stream_comprese.subscribe(function(message) {
            //document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
            document.getElementById('my_image').src = "data:image/bgr8;base64," + message.data;
          });
          
        }else if(uav_type == "px4"){
          // PX4
  
          var buttonLoad = document.createElement("button");
          buttonLoad.setAttribute("class", "btn btn-default icon-front icon icon-upload");
          // buttonLoad.style.marginLeft = '25%';
          //buttonLoad.addEventListener("click", function () {
          //  loadMissionToUAV(uav_ns);				
          //  });			
          //cell5.appendChild(buttonLoad);
          var buttonFlyToUav = document.createElement("button");
          buttonFlyToUav.setAttribute("class", "btn btn-default icon-front");
          buttonFlyToUav.innerHTML = "Fly!";			
          //buttonFlyToUav.addEventListener("click", function () {
          //  commandMissionToUAV(uav_ns);				
          //      });			
          //cell5.appendChild(buttonFlyToUav);
  
  
          uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/global_position/global',
            messageType : 'sensor_msgs/NavSatFix'
          });
  
          uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/global_position/compass_hdg',
            messageType : 'std_msgs/Float64'
          });
  
          uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/local_position/velocity_local',
            messageType : 'geometry_msgs/TwistStamped'
          });
  
          uav_list[cur_uav_idx].listener_alt = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/altitude',
            messageType : 'mavros_msgs/Altitude'
          });
  
          uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/battery',
            messageType : 'sensor_msgs/BatteryState'
          });
  
          uav_list[cur_uav_idx].listener.subscribe(function(msg) {
            let id_uav = cur_uav_idx;
            dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.latitude,longitude:msg.longitude,altitude:msg.altitude,deviceTime:"2023-03-09T22:12:44.000+00:00"})); 
            //uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
            //uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);
  
            // Fill info cell. Lo hacemos aqui para que se actualice cada vez que recibimos un mensaje en este topic
            //var showData = document.getElementById(uav_ns).cells;
            // podemos poner aqui una condicion para que solo muestre connected al principio (por ejemplo: si innerHTML == "Waiting uav", o no hacerlo si es == "Ready to launch")
            //showData[5].innerHTML = "Connected";
  
            //var buttonMarkerSelect = '<img src="../assets/css/images/px4-marker'+marker_n.toString()+'.png " width="24" height="24" />';
            //buttonMarker.innerHTML = buttonMarkerSelect;
                      
            //if (uav_list[cur_uav_idx].watch_bound){
            //  uav_list[cur_uav_idx].marker.setOpacity(1);
              //map.fitBounds(L.latLngBounds([ uav_list[cur_uav_idx].marker.getLatLng() ]));  
            //  uav_list[cur_uav_idx].watch_bound = false
            //}
          });
  
          uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
            //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
          });
  
          uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
            //var showData = document.getElementById(uav_ns).cells;
            //showData[1].innerHTML = (message.relative).toFixed(2);
                    
          });
  
          uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
            //var showData = document.getElementById(uav_ns).cells;
            // showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
          });
  
          uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
            //var showData = document.getElementById(uav_ns).cells;
            //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
          });
        } else {
  
          //EXT (FUVEX)
  
          var buttonLoad = document.createElement("button");
          buttonLoad.setAttribute("class", "btn btn-default icon-front icon icon-upload");
          // buttonLoad.style.marginLeft = '25%';
          //buttonLoad.addEventListener("click", function () {
          //  loadMissionToExt(uav_ns);				
          //});			
          //cell5.appendChild(buttonLoad);
          var buttonFlyToUav = document.createElement("button");
          buttonFlyToUav.setAttribute("class", "btn btn-default icon-front");
          buttonFlyToUav.setAttribute("id", "ReadyToFly"+uav_ns);
          // buttonFlyToUav.style.marginLeft = '50%';
          buttonFlyToUav.innerHTML = "X";			
          //buttonFlyToUav.addEventListener("click", function () {
          //  commandMissionToEXT(uav_ns);				
          //  });			
          //cell5.appendChild(buttonFlyToUav);
  
  
          uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/global_position/global',
            messageType : 'sensor_msgs/NavSatFix'
          });
  
          uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/global_position/compass_hdg',
            messageType : 'std_msgs/Float64'
          });
  
          uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/local_position/velocity_local',
            messageType : 'geometry_msgs/TwistStamped'
          });
  
          uav_list[cur_uav_idx].listener_alt = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/altitude',
            messageType : 'mavros_msgs/Altitude'
          });
  
          uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
            ros : ros,
            name : uav_ns+'/mavros/battery',
            messageType : 'sensor_msgs/BatteryState'
          });
  
          uav_list[cur_uav_idx].listener.subscribe(function(message) {
            uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
            uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);
  
            var buttonMarkerSelect = '<img src="../assets/css/images/ext-marker'+marker_n.toString()+'.png " width="24" height="24" />';
            buttonMarker.innerHTML = buttonMarkerSelect;
            
            if (uav_list[cur_uav_idx].watch_bound){
              uav_list[cur_uav_idx].marker.setOpacity(1);
              //map.fitBounds(L.latLngBounds([ uav_list[cur_uav_idx].marker.getLatLng() ]));  
              uav_list[cur_uav_idx].watch_bound = false
            }
          });
  
          uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
            uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
          });
  
          uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
            var showData = document.getElementById(uav_ns).cells;
              showData[1].innerHTML = (message.relative).toFixed(2);										
          });
  
          uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
            var showData = document.getElementById(uav_ns).cells;
              showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
          });
  
          uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
            var showData = document.getElementById(uav_ns).cells;
              showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
          });
        }
  
        console.log("\nLa Lista de uav's es: ");
        uav_list.forEach(function(uav, indice, array) {
          console.log(uav, indice);
        })
  
        // alert("Total de drones conectados: "+uav_list.length+"\n Dron añadido a la lista:\n\nNombre: "+uavAdded.name +"\n Tipo: "+ uavAdded.type);
        updateStatusWindow(uavAdded.name + " added. Type: "+ uavAdded.type);
  
      // showAddUav();
      }else{
        alert("\nRos no está conectado.\n\n Por favor conéctelo primero.")
  
      } // if ros connected
  
  
    }

    function loadMissionToUAV(ns){
      let uav_idx = uav_list.findIndex(element => element.name == ns)
      let wp_command = []
      uav_list[uav_idx].wp_list.forEach(function prepare_wp(item,idx,arr){
        let pos = new ROSLIB.Message({
          latitude: item[0], 
          longitude: item[1],
          altitude: item[2]
        });
  
        wp_command.push(pos)
      })
  
      if(uav_list[uav_idx].type == "dji"){
        var missionClient = new ROSLIB.Service({
          ros : ros,
          name : ns + '/dji_control/configure_mission',
          serviceType : 'aerialcore_common/ConfigMission'
        });
      }else{
        var missionClient = new ROSLIB.Service({
          ros : ros,
          name : ns + '/mission/new',
          serviceType : 'aerialcore_common/ConfigMission'
        });
      }
  
      var request = new ROSLIB.ServiceRequest({
        type : "waypoint",
        waypoint: wp_command,
        radius : 0,
        maxVel:	3,
        idleVel: 2,
        yawMode: 0,
        traceMode: 0,
        finishAction: mode_landing
      });
  
      missionClient.callService(request, function(result) {
        console.log('Result for service call on '
        + missionClient.name
        + ': '
        + result.success);
      }, function(result) {
        console.log('Error:'
        + result);
        alert('Error:'
        + result);
      });
  
      // let flyButton = document.getElementById("commandMission");
      // if(flyButton.style.cssText == "visibility: none;"){
      // 	flyButton.style.cssText = "visibility: hidden;"
      // }else{
      // 	flyButton.style.cssText = "visibility: none;"
      // }
      updateStatusWindow("Loadding mission to: " + ns);
    }

  return (
    <div style={{    width: '100%',height: '100%'}}>    
      <RosContext.Provider value={{rosConnect , rosState,openMision,connectAddUav}}>
        {children}
      </RosContext.Provider>
    </div>
  )
}
