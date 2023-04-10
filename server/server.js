const express = require("express")
const app = express()
const port = 4000
const cors = require("cors")


app.use(express.urlencoded({extended:true}))
app.use(express.json())
app.use(cors)

var rosState = False
let uav_list = [];

function setrosState(state){
    rosState = state
}

const rosConnect = () =>{
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

      let cur_uav_idx = String(Object.values(devices).length)

      dispatch(devicesActions.update({id:cur_uav_idx,name:uav_ns,category:uav_type,status:'online'}));

      
      let uavAdded = { name : uav_ns, type : uav_type, watch_bound : true, wp_list : [] , listener : "", listener_hdg : "", listener_alt : "", listener_vel : "", listener_bat : "",bag : false};
      uav_list.push(uavAdded);

      // Subscribing
      // DJI 
      if(uav_type == "dji"){

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
          
        });
        uav_list[cur_uav_idx].listenerov.subscribe(function(msg) {
          //let id_uav = cur_uav_idx;
          //dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.x,longitude:msg.y,altitude:msg.z,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));          
        });

        uav_list[cur_uav_idx].listener_hdg.subscribe(function(msg) {
          let id_uav = cur_uav_idx;
          dispatch(dataActions.updatePosition({deviceId:id_uav,course:msg.data+90}));//uav_list[cur_uav_idx].marker.setRotationAngle(message.data+90);
        });				
        
        uav_list[cur_uav_idx].listener_vel.subscribe(function(msg) {
          let id_uav = cur_uav_idx;// var showData = document.getElementById(uav_ns).cells;
         dispatch(dataActions.updatePosition({deviceId:id_uav,speed:Math.sqrt(Math.sqrt(Math.pow(msg.vector.x,2) + Math.pow(msg.vector.y,2)).toFixed(2))}));// showData[2].innerHTML = Math.sqrt(Math.pow(message.vector.x,2) + Math.pow(message.vector.y,2)).toFixed(2);
        });				
        
        uav_list[cur_uav_idx].listener_bat.subscribe(function(msg) {
          let id_uav = cur_uav_idx//var showData = document.getElementById(uav_ns).cells;
          dispatch(dataActions.updatePosition({deviceId:id_uav,batteryLevel:msg.percentage}));// showData[3].innerHTML = message.percentage + "%";
        });

        uav_list[cur_uav_idx].camera_stream_comprese.subscribe(function(msg) {
          let id_uav = cur_uav_idx;
          dispatch(dataActions.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data}));//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
          //document.getElementById('my_image').src = "data:image/bgr8;base64," + message.data;
        });
        
      }else if(uav_type == "px4"){

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
        });

        uav_list[cur_uav_idx].listener_hdg.subscribe(function(msg) {
          let id_uav = cur_uav_idx;
          dispatch(dataActions.updatePosition({deviceId:id_uav,course:msg.data}));//uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
        });

        uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
          //var showData = document.getElementById(uav_ns).cells;
          //showData[1].innerHTML = (message.relative).toFixed(2);   
        });

        uav_list[cur_uav_idx].listener_vel.subscribe(function(msg) {
          let id_uav = cur_uav_idx;//var showData = document.getElementById(uav_ns).cells;
          dispatch(dataActions.updatePosition({deviceId:id_uav,speed:Math.sqrt(Math.pow(msg.twist.linear.x,2) + Math.pow(msg.twist.linear.y,2)).toFixed(2)}));// showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
        });

        uav_list[cur_uav_idx].listener_bat.subscribe(function(msg) {
          let id_uav = cur_uav_idx;//var showData = document.getElementById(uav_ns).cells;
          dispatch(dataActions.updatePosition({deviceId:id_uav,batteryLevel:(msg.percentage*100).toFixed(0)}));//  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
        });
      } else {
        //EXT (FUVEX)
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
      notification('success', uavAdded.name + " added. Type: "+ uavAdded.type);
    }else{
      alert("\nRos no está conectado.\n\n Por favor conéctelo primero.")
    } 
  }




app.get("/",cors(),async(req, res) => {
    res.send("this is working")
})

app.get('/api/notes', (request, response) => {
    response.json(notes)
  })

  app.get('/api/notes/:id', (request, response) => {
    const id = request.params.id
    const note = notes.find(note => note.id === id)
    response.json(note)
  })


app.listen(port, ()=>{
    console.log(`listen at http://localhost:${port}`)
})