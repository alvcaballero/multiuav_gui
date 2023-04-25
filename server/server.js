const data = require ("./data")
const express = require("express")
const cors = require("cors")


const WebSocket = require("ws");
const ROSLIB = require('roslib');

const uuidv4 = require('uuid').v4;


const port = 4000

const app = express()
app.set("port", port);
app.use(cors());
app.use(express.json());


//const expressWs = require('express-ws')(app);
const server = require('http').createServer(app);
//app.use(express.static(path.join(__dirname, "./public")));

const clients = {};
//console.log(server)
//const wsServer = new WebSocketServer({  httpServer:server,autoAcceptConnections: false });
const wss = new WebSocket.Server({ server:server });


wss.on('connection', function connection(ws) {

  console.log("newclient")
  ws.on('error', console.error);

  ws.on('message', function message(data) {
    console.log('received: %s', data);
  });
  const interval = setInterval(() => {
    ws.send(JSON.stringify({ devices: data.state.devices , positions: data.state.positions}));
  }, 200);
});



// Iniciamos el servidor en el puerto establecido por la variable port (3000)
server.listen(app.get('port'), () =>{console.log('Servidor iniciado en el puerto: ' +app.get('port'));})

var rosState = {state:'fail',msg:"init msg"};
let uav_list = [];
let ros = "";



function setrosState(state){
    rosState = state;
}

async function rosConnect(){
    if(rosState.state != 'connect'){
      ros = new ROSLIB.Ros({url : 'ws://localhost:9090'});
      ros.on('connection', function() {
        console.log('Connected to websocket server.');
        setrosState({state:'connect',msg:"Conectado a ROS"});
      });
      ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        setrosState({state:'error',msg:"No se ha posido conectar a ROS"});
      });
      ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        setrosState({state:'disconnect',msg:"Desconectado a ROS"});
      });
    }else{
      for (var i = 0; i < uav_list.length; i++) {
        uav_list[i].listener.unsubscribe();
      }
      uav_list = [];
      ros.close();
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
    if(rosState.state ==='connect'){

      const topiclist = await getTopics2();
      console.log("despues del await")

      console.log("Conectando Uav");
      console.log(topiclist)



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
        console.log("Dispositivo no encontrado"+uav_ns);
        return {state:'fail',msg:`Dispositivo no encontrado+${uav_ns}`};
      }
      if( repeat_device == true){ 
        console.log("Dispositivo se encuentra registrado "+uav_ns);
        return {state:'fail',msg:`Dispositivo se encuentra registrado${uav_ns}`};
      }

      let cur_uav_idx = String(Object.values(data.state.devices).length)

      data.updatedevice({id:cur_uav_idx,name:uav_ns,category:uav_type,status:'online'})

      let uavAdded = { name : uav_ns, type : uav_type, watch_bound : true, wp_list : [] , listener : "", listener_hdg : "", listener_alt : "", listener_vel : "", listener_bat : "",listener_cam : "",bag : false};

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

        uav_list[cur_uav_idx].listener_cam = new ROSLIB.Topic({
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
          data.updatePosition({id : msg.header.seq,deviceId:id_uav,  latitude:msg.latitude,longitude:msg.longitude, altitude:msg.altitude,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"});
          
        });
        uav_list[cur_uav_idx].listenerov.subscribe(function(msg) {
          //let id_uav = cur_uav_idx;
          //dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.x,longitude:msg.y,altitude:msg.z,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));          
        });

        uav_list[cur_uav_idx].listener_hdg.subscribe(function(msg) {
          let id_uav = cur_uav_idx;
          data.updatePosition({deviceId:id_uav,course:msg.data+90});//uav_list[cur_uav_idx].marker.setRotationAngle(message.data+90);
        });				
        
        uav_list[cur_uav_idx].listener_vel.subscribe(function(msg) {
          let id_uav = cur_uav_idx;// var showData = document.getElementById(uav_ns).cells;
          data.updatePosition({deviceId:id_uav,speed:Math.sqrt(Math.sqrt(Math.pow(msg.vector.x,2) + Math.pow(msg.vector.y,2)).toFixed(2))});// showData[2].innerHTML = Math.sqrt(Math.pow(message.vector.x,2) + Math.pow(message.vector.y,2)).toFixed(2);
        });				
        
        uav_list[cur_uav_idx].listener_bat.subscribe(function(msg) {
          let id_uav = cur_uav_idx//var showData = document.getElementById(uav_ns).cells;
          data.updatePosition({deviceId:id_uav,batteryLevel:msg.percentage});// showData[3].innerHTML = message.percentage + "%";
        });

        uav_list[cur_uav_idx].listener_cam.subscribe(function(msg) {
          let id_uav = cur_uav_idx;
          data.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data});//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
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
        uav_list[cur_uav_idx].listener_cam = new ROSLIB.Topic({
            ros : ros,
           name : uav_ns+'/video_stream_compress',
            messageType : 'sensor_msgs/CompressedImage'
        });

        uav_list[cur_uav_idx].listener.subscribe(function(msg) {
          let id_uav = cur_uav_idx;
          data.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.latitude,longitude:msg.longitude,altitude:msg.altitude,deviceTime:"2023-03-09T22:12:44.000+00:00"}); 
        });

        uav_list[cur_uav_idx].listener_hdg.subscribe(function(msg) {
          let id_uav = cur_uav_idx;
          data.updatePosition({deviceId:id_uav,course:msg.data});//uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
        });

        uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
          //var showData = document.getElementById(uav_ns).cells;
          //showData[1].innerHTML = (message.relative).toFixed(2);   
        });

        uav_list[cur_uav_idx].listener_vel.subscribe(function(msg) {
          let id_uav = cur_uav_idx;//var showData = document.getElementById(uav_ns).cells;
          data.updatePosition({deviceId:id_uav,speed:Math.sqrt(Math.pow(msg.twist.linear.x,2) + Math.pow(msg.twist.linear.y,2)).toFixed(2)});// showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
        });

        uav_list[cur_uav_idx].listener_bat.subscribe(function(msg) {
          let id_uav = cur_uav_idx;//var showData = document.getElementById(uav_ns).cells;
          data.updatePosition({deviceId:id_uav,batteryLevel:(msg.percentage*100).toFixed(0)});//  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
        });
        uav_list[cur_uav_idx].listener_cam.subscribe(function(msg) {
            let id_uav = cur_uav_idx;
            dispatch(dataActions.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data}));//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;//document.getElementById('my_image').src = "data:image/bgr8;base64," + message.data;
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
          //uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
          //uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);
        });

        uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
          //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
        });

        uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
          //var showData = document.getElementById(uav_ns).cells;
          //  showData[1].innerHTML = (message.relative).toFixed(2);										
        });

        uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
          //var showData = document.getElementById(uav_ns).cells;
          //  showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
        });

        uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
          //var showData = document.getElementById(uav_ns).cells;
          //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
        });
      }

      console.log("\nLa Lista de uav's es: ");
      uav_list.forEach(function(uav, indice, array) {
        console.log(uav, indice);
      })
      console.log('success', uavAdded.name + " added. Type: "+ uavAdded.type);
      return {state:'success',msg:"conectado Correctamente"};
    }else{
      console.log("\nRos no está conectado.\n\n Por favor conéctelo primero.")
      return {state:'fail',msg:"Ros no está conectado"};
    } 
  }

  function loadMission(mission){ 
    console.log("  load  - mission")
    console.log(mission)
    console.log("uav list")
    console.log(uav_list)
    let cur_roster = []
    let cur_ns = ""
    let mode_yaw = 0;
    let mode_landing =0;
    uav_list.forEach(function prepare_wp(item,idx,arr){
      cur_ns = item.name
      cur_roster.push(cur_ns);
      if (item.type !== "ext"){
        let wp_command = [];
        let yaw_pos =[];
        Object.values(mission).forEach(route => {
          if(route['name'] == cur_ns){
            console.log("route")
            console.log(route)
            Object.values(route['wp']).forEach(
              function prepare_wp(item,idx,arr){
                let pos = new ROSLIB.Message({
                  latitude: item[0], 
                  longitude: item[1],
                  altitude: item[2]  
                });
                
              yaw_pos.push(item[3])
              wp_command.push(pos);
            });
            if (route.attributes.hasOwnProperty("mode_landing")){
              mode_landing =  route.attributes["mode_landing"];  
            }
            if (route.attributes.hasOwnProperty("mode_yaw")){
              mode_yaw =  route.attributes["mode_yaw"];  
            }
          }
        })
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
        console.log("request")
        console.log(request)

        missionClient.callService(request, function(result) {
          console.log('load mission'+ missionClient.name+': '+result.success);
            if(result.success){
              return {state:'success',msg:"Loadding mission to" + cur_roster+" ok"};//notification('success',"Load mission to:" + cur_roster + " ok");
            } else{
              return {state:'fail',msg:"Load mission to:" + cur_roster + " fail"};//notification('danger',"Load mission to:" + cur_roster + " fail");
            }
        }, function(result) {
          console.log('Error:'+ result);
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
    return {state:'success',msg:"Loadding mission to" + cur_roster};//notification('success',"Loadding mission to: " + cur_roster);
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
              return {state:'success',msg:"Start mission:" + cur_roster + " ok"};//notification('success',"Start mission:" + cur_roster + " ok");
            } else{
              return {state:'fail',msg:"Start mission:" + cur_roster + " FAIL!"};//notification('danger',"Start mission:" + cur_roster + " FAIL!");
            }
          });
        } else {
          console.log(uav.name);
          //commandMissionToEXT(uav.name);
        }
      });
      return {state:'success',msg:"Commanding mission to: " + cur_roster};//notification('success',"Commanding mission to: " + cur_roster);
    } else {
      console.log("Mission canceled")
      return {state:'fail',msg:"Mission canceled"};
    }
  }

  function disConnectAddUav(uav_ns){
    Key_listener = Object.keys(data.state.devices).find(element => element.includes("listener"))
		let cur_uav_idx = uav_ns//uav_list.length-1;
		if (uav_list.length != 0){
      Key_listener.forEach(element => {
        uav_list[cur_uav_idx][element].unsubscribe();
      })
			uav_list.pop();
			console.log("Último dron eliminado de la lista");
			if (uav_list.length != 0){
				console.log("\nLa Lista de uav actualizada es: ");
				uav_list.forEach(function(elemento, indice, array) {
				console.log(elemento, indice);
				})
			}else{
				console.log("No quedan drones en la lista");
			}
    return {state:'success',msg:"Se ha eliminado el "+cur_uav_idx};//notification('success',"Commanding mission to: " + cur_roster);
		}else{
			return {state:'success',msg:"no quedan UAV de la lista"};
		}
	}




  function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  app.get('/', (request, response) => {
    response.send('<h1>Hello World!</h1>')
  })
  

app.get('/api/devices', (req, res) => {
  console.log('devicesget')
  res.json(data.state.devices)
});

app.get('/api/positions', (req, res) => {
  console.log('positionsget')
  res.json(data.state.positions)
});
// camera
app.get('/api/media/:deviceid', (req, res) => {
  console.log('cameraget')
  console.log('cameraget'+req.params.deviceid)
  if (data.state.camera[req.params.deviceid]){
    res.json(data.state.camera[req.params.deviceid])
  }else{
    res.status(404).end()
  }
});

app.post('/api/devices',async function(req,res){
  console.log('devicespost')
  console.log(req.body.uav_ns)
  let myresponse = await connectAddUav(req.body.uav_ns,req.body.uav_type)
  return res.json(myresponse);
});

app.post('/api/loadmission',async function(req,res){
  console.log('loadmission-post')
  //console.log(req.body.uav_ns)
  let myresponse = await loadMission(req.body.mission); 
  return res.json(myresponse);
});

app.post('/api/commandmission',async function(req,res){
  console.log('loadmission-post')
  //console.log(req.body.uav_ns)
  let myresponse = await commandMission();
  return res.json(myresponse);
});

app.post('/api/disconectdevice',async function(req,res){
  console.log('loadmission-post')
  //console.log(req.body.uav_ns)
  let myresponse = await disConnectAddUav(req.body.uav_ns);
  return res.json(myresponse);
});

app.post('/api/rosConnect',async function(req,res){
  console.log('rosconect')
  rosConnect();
  await sleep(200);
  return res.json(rosState);
});

app.get('/api/rosConnect',function(req,res){
  console.log('getrosconnect');
  return res.json(rosState);
});


app.get('/api/topics',async function(req,res){
  console.log('rosTopic');
  const topiclist = await getTopics2();
  return res.json(topiclist);
});
