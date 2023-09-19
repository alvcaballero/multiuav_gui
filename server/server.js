//https://www.youtube.com/watch?v=gnM3Ld6_upE-- REVISAR
//https://medium.com/agora-io/how-does-webrtc-work-996748603141
//Server-Sent Events vs. WebSockets
const turf = require("@turf/turf");
const data = require("./data");
const path = require("path");
const express = require("express");
const cors = require("cors");
const fs = require("fs");
const YAML = require("yaml");

const WebSocket = require("ws");
const ROSLIB = require("roslib");

const app = express();
const port = 4000;
app.set("port", port);
app.use(cors());
app.use(express.json());
app.use(express.static(path.resolve(__dirname, "../build")));
const server = require("http").createServer(app);
const wss = new WebSocket.Server({
  path: "/api/socket",
  server: server,
});

var devices_msg = {};
try {
  let fileContents = fs.readFileSync(
    path.resolve(__dirname, "./devices_msg.yaml"),
    "utf8"
  );
  devices_msg = YAML.parse(fileContents);
  console.log("load devices msg");
} catch (e) {
  console.log(e);
}

var devices_init = {};
try {
  let fileContents = fs.readFileSync(
    path.resolve(__dirname, "./devices_init.yaml"),
    "utf8"
  );
  devices_init = YAML.parse(fileContents);
  console.log("load devices msg");
} catch (e) {
  console.log(e);
}

var rosState = { state: "disconnect", msg: "init msg" };
let uav_list = [];
let ros = "";

function setrosState(state) {
  rosState = state;
}

//const uuidv4 = require('uuid').v4;

wss.on("connection", function connection(ws) {
  console.log("newclient");
  ws.on("error", console.error);

  ws.on("message", function message(data) {
    console.log("received: %s", data);
  });
  ws.send(
    JSON.stringify({
      positions: Object.values(data.state.positions),
      camera: Object.values(data.state.camera),
      server: { rosState: rosState.state },
      devices: Object.values(data.state.devices),
    })
  );

  const interval = setInterval(() => {
    ws.send(
      JSON.stringify({
        positions: Object.values(data.state.positions),
        camera: Object.values(data.state.camera),
      })
    );
  }, 200);

  const interval_server = setInterval(() => {
    ws.send(
      JSON.stringify({
        server: { rosState: rosState.state },
        devices: Object.values(data.state.devices),
      })
    );
  }, 1000);
});
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"
const interval = setInterval(function ping() {
  wss.clients.forEach(function each(ws) {
    //if (ws.isAlive === false) return ws.terminate();
    //ws.isAlive = false;
    //ws.ping();
  });
}, 30000);

wss.on("close", function close() {
  clearInterval(interval);
});

server.listen(app.get("port"), () => {
  console.log("Servidor iniciado en el puerto: " + app.get("port"));
});

const CheckDeviceOnline = setInterval(() => {
  let currentTime = new Date();
  let checkdevices = Object.keys(data.state.devices);
  checkdevices.forEach((element) => {
    //console.log(data.state.devices[element])
    //console.log(currentTime)
    if (currentTime - data.state.devices[element]["lastUpdate"] < 5000) {
      data.state.devices[element]["status"] = "online";
    } else {
      data.state.devices[element]["status"] = "offline";
    }
  });
}, 5000);

async function rosConnect() {
  console.log("try to connect to ros");
  if (rosState.state != "connect") {
    ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
    ros.on("connection", function () {
      console.log("ROS Connected to websocket server.");
      setrosState({ state: "connect", msg: "Conectado a ROS" });
      connectAllUAV();
    });
    ros.on("error", function (error) {
      console.log("ROS Error connecting to websocket server: ", error);
      setrosState({ state: "error", msg: "No se ha posido conectar a ROS" });
    });
    ros.on("close", function () {
      console.log("ROS Connection to websocket server closed.");
      setrosState({ state: "disconnect", msg: "Desconectado a ROS" });
    });
  } else {
    for (var i = 0; i < uav_list.length; i++) {
      uav_list[i].listener.unsubscribe();
    }
    uav_list = [];
    ros.close();
  }
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
rosConnect();

async function connectAllUAV() {
  for (let element of devices_init.init) {
    console.log(element);
    let uno = await connectAddUav(element);
    console.log("finish to add ------------");
  }
}

const autoconectRos = setInterval(() => {
  if (rosState.state != "connect") {
    console.log("try to connect ros");
    rosConnect();
  }
}, 30000);

const getTopics2 = () => {
  var topicsClient = new ROSLIB.Service({
    ros: ros,
    name: "/rosapi/topics",
    serviceType: "rosapi/Topics",
  });

  var request = new ROSLIB.ServiceRequest();

  return new Promise((resolve, rejects) => {
    topicsClient.callService(request, function (result) {
      resolve(result.topics);
    });
  });
};

async function connectAddUav(device) {
  if (rosState.state === "connect") {
    uav_ns = device.name;
    uav_type = device.category;
    console.log("");

    const topiclist = await getTopics2();
    console.log("Conectando Uav");
    console.log("name" + uav_ns + "  type" + uav_type);
    console.log(topiclist);

    let find_device = false,
      repeat_device = false;
    topiclist.forEach((element) => {
      find_device = element.includes(uav_ns) || find_device;
    });
    find_device = true;
    if (uav_list.length > 0) {
      uav_list.forEach((element) => {
        console.log(element);
        if (element) {
          repeat_device = element.name == uav_ns ? true : false;
        }
      });
    }

    if (find_device == false || uav_ns == "") {
      console.log("Dispositivo no encontrado" + uav_ns);
      return { state: "fail", msg: `Dispositivo no encontrado+${uav_ns}` };
    }
    if (repeat_device == true) {
      console.log("Dispositivo ya se encuentra registrado " + uav_ns);
      return {
        state: "fail",
        msg: `Dispositivo ya se encuentra registrado ${uav_ns}`,
      };
    }

    let cur_uav_idx = String(Object.values(data.state.devices).length);

    data.updatedevice({
      id: cur_uav_idx,
      name: device.name,
      category: device.category,
      ip: device.ip,
      camera: device.camera,
      status: "online",
      lastUpdate: null,
    });

    for (let i = 0; i < device.camera.length; i = i + 1) {
      if (device.camera[i]["type"] == "WebRTC") {
        await fetch(
          `http://localhost:9997/v2/config/paths/add/${device.name}_${device.camera[i].source}`,
          {
            method: "POST",
            body: JSON.stringify({
              source: `rtsp://${device.ip}:8554/${device.camera[i].source}`,
            }),
            headers: {
              "Content-Type": "application/json",
            },
          }
        );
      }
    }

    let uavAdded = {
      name: uav_ns,
      type: uav_type,
      watch_bound: true,
      wp_list: [],
      bag: false,
    };

    console.log(data.state.devices);
    uav_list.push(uavAdded);

    // Subscribing
    // create subcribin mesage
    Object.keys(devices_msg[uav_type]["topics"]).forEach((element) => {
      console.log(element);
      uav_list[cur_uav_idx]["listener_" + element] = new ROSLIB.Topic({
        ros: ros,
        name: uav_ns + devices_msg[uav_type]["topics"][element]["name"], //'/dji_osdk_ros/rtk_position',
        messageType: devices_msg[uav_type]["topics"][element]["messageType"],
      });
    });
    // DJI
    if (uav_type == "dji" || uav_type == "dji_M300") {
      uav_list[cur_uav_idx].listener_position.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updatePosition({
          id: msg.header.seq,
          deviceId: id_uav,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: "2023-03-09T22:12:44.000+00:00",
        });
      });

      uav_list[cur_uav_idx].listener_sensor_height.subscribe(function (msg) {
        //Altitud de ultrasonico
        //let id_uav = cur_uav_idx;
        //data.updatePosition({deviceId:id_uav,altitude:msg.data});
      });

      uav_list[cur_uav_idx].listener_vo_position.subscribe(function (msg) {
        //let id_uav = cur_uav_idx;
        //console.log(msg)
        //dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:id_uav,latitude:msg.x,longitude:msg.y,altitude:msg.z,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));
      });

      uav_list[cur_uav_idx].listener_IMU.subscribe(function (msg) {
        //https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        let id_uav = cur_uav_idx;
        let q = msg.orientation;
        let yaw =
          Math.atan2(
            2.0 * (q.x * q.y + q.w * q.z),
            -1 + 2 * (q.w * q.w + q.x * q.x)
          ) * -1;
        data.updatePosition({ deviceId: id_uav, course: 90 + yaw * 57.295 });
      });

      uav_list[cur_uav_idx].listener_speed.subscribe(function (msg) {
        let id_uav = cur_uav_idx; // var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({
          deviceId: id_uav,
          speed: Math.sqrt(
            Math.sqrt(
              Math.pow(msg.vector.x, 2) + Math.pow(msg.vector.y, 2)
            ).toFixed(2)
          ),
        }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.vector.x,2) + Math.pow(message.vector.y,2)).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_battery.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({ deviceId: id_uav, batteryLevel: msg.percentage }); // showData[3].innerHTML = message.percentage + "%";
      });

      uav_list[cur_uav_idx].listener_camera.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //console.log("dato camara"+ id_uav + "--"+ msg.data)
        data.updateCamera({ deviceId: id_uav, camera: msg.data }); //data.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data});//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
      });
    } else if (uav_type == "px4") {
      uav_list[cur_uav_idx].listener_position.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updatePosition({
          id: msg.header.seq,
          deviceId: id_uav,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: "2023-03-09T22:12:44.000+00:00",
        });
      });

      uav_list[cur_uav_idx].listener_hdg.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updatePosition({ deviceId: id_uav, course: msg.data }); //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
      });

      uav_list[cur_uav_idx].listener_sensor_height.subscribe(function (
        message
      ) {
        //var showData = document.getElementById(uav_ns).cells;
        //showData[1].innerHTML = (message.relative).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_speed.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({
          deviceId: id_uav,
          speed: Math.sqrt(
            Math.pow(msg.twist.linear.x, 2) + Math.pow(msg.twist.linear.y, 2)
          ).toFixed(2),
        }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_battery.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({
          deviceId: id_uav,
          batteryLevel: (msg.percentage * 100).toFixed(0),
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });

      uav_list[cur_uav_idx].listener_state_machine.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({
          deviceId: id_uav,
          protocol: msg.airframe_type,
          mission_state: msg.mission_state,
          wp_reached: msg.wp_reached,
          uav_state: msg.uav_state,
          landed_state: msg.landed_state,
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });

      uav_list[cur_uav_idx].listener_camera.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updateCamera({ deviceId: id_uav, camera: msg.data }); //data.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data});//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;//document.getElementById('my_image').src = "data:image/bgr8;base64," + message.data;
      });

      uav_list[cur_uav_idx].listener_threat.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updatePosition({ deviceId: id_uav, threat: msg.data });
      });

      //let ipdevice = await Getservicehost(uav_ns+'/mavros/mission/clear');
      //console.log("ip de uav"+ ipdevice);
      //data.updatedeviceIP({id: cur_uav_idx,ip:ipdevice});
    } else if (uav_type == "fuvex") {
      //EXT (FUVEX)

      uav_list[cur_uav_idx].listener_position.subscribe(function (message) {
        //uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
        //uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);
      });

      uav_list[cur_uav_idx].listener_hdg.subscribe(function (message) {
        //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
      });

      uav_list[cur_uav_idx].listener_sensor_height.subscribe(function (
        message
      ) {
        //var showData = document.getElementById(uav_ns).cells;
        //  showData[1].innerHTML = (message.relative).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_speed.subscribe(function (message) {
        //var showData = document.getElementById(uav_ns).cells;
        //  showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_battery.subscribe(function (message) {
        //var showData = document.getElementById(uav_ns).cells;
        //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });
    } else if (uav_type == "catec") {
      uav_list[cur_uav_idx].listener_position.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updatePosition({
          id: msg.header.seq,
          deviceId: id_uav,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: "2023-03-09T22:12:44.000+00:00",
        });
      });

      uav_list[cur_uav_idx].listener_hdg.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updatePosition({ deviceId: id_uav, course: msg.data }); //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
      });

      uav_list[cur_uav_idx].listener_sensor_height.subscribe(function (
        message
      ) {
        //var showData = document.getElementById(uav_ns).cells;
        //showData[1].innerHTML = (message.relative).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_speed.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({
          deviceId: id_uav,
          speed: Math.sqrt(
            Math.pow(msg.twist.linear.x, 2) + Math.pow(msg.twist.linear.y, 2)
          ).toFixed(2),
        }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_battery.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({
          deviceId: id_uav,
          batteryLevel: (msg.percentage * 100).toFixed(0),
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });

      uav_list[cur_uav_idx].listener_state_machine.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        data.updatePosition({
          deviceId: id_uav,
          protocol: "catec",
          mission_state: "0",
          wp_reached: "0",
          uav_state: "ok",
          landed_state: msg.data,
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });

      uav_list[cur_uav_idx].listener_threat.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        data.updatePosition({ deviceId: id_uav, threat: msg.data });
      });
    }

    console.log("\nLa Lista de uav's es: ");
    uav_list.forEach(function (uav, indice, array) {
      console.log(uav, indice);
    });
    console.log("success", uavAdded.name + " added. Type: " + uavAdded.type);
    return { state: "success", msg: "conectado Correctamente" };
  } else {
    console.log("\nRos no está conectado.\n\n Por favor conéctelo primero.");
    return { state: "fail", msg: "Ros no está conectado" };
  }
}

const Getservicehost = (nameService) => {
  let servicehost = new ROSLIB.Service({
    ros: ros,
    name: "/rosapi/service_host",
    serviceType: "rosapi/ServiceHost",
  });

  let request = new ROSLIB.ServiceRequest({ service: nameService });

  return new Promise((resolve, rejects) => {
    servicehost.callService(request, function (result) {
      resolve(result.host);
    });
  });
};
const Getlistmaster = () => {
  let servicemaster = new ROSLIB.Service({
    ros: ros,
    name: "/master_discovery/list_masters",
    serviceType: "multimaster_msgs_fkie/DiscoverMasters",
  });

  let request = new ROSLIB.ServiceRequest();

  return new Promise((resolve, rejects) => {
    servicemaster.callService(request, function (result) {
      console.log("masterip -- " + result.length);
      resolve(result);
    });
  });
};

function threatUAV(uav_id) {
  let uavname = data.get_device_ns(uav_id);
  threadmessage = new ROSLIB.Service({
    ros: ros,
    name: uavname + "/threat_confirmation",
    serviceType: "std_srvs/Trigger",
  });
  let request = new ROSLIB.ServiceRequest({});
  threadmessage.callService(
    request,
    function (result) {
      console.log("send threat");
      console.log(result);
      if (result.success) {
        return {
          state: "success",
          msg: "threat to" + uavname + " ok" + result.message,
        }; //notification('success',"Load mission to:" + cur_roster + " ok");
      } else {
        return {
          state: "fail",
          msg: "threat to:" + uavname + " fail" + result.message,
        }; //notification('danger',"Load mission to:" + cur_roster + " fail");
      }
    },
    function (result) {
      console.log("Error:" + result);
      return {
        state: "fail",
        msg: "Sincronize to:" + uavname + " Error:" + result,
      };
    }
  );
}

function sincronizeUAV(uav_id) {
  console.log("sincronize uav_id" + uav_id);
  let uavname = data.get_device_ns(uav_id);
  let uavcategory = data.get_device_category(uav_id);
  console.log(
    "sincronize uav_id" +
      uav_id +
      "--" +
      uavcategory +
      "--" +
      devices_msg[uavcategory]["services"]["sincronize"]["name"]
  );

  sincronizemessage = new ROSLIB.Service({
    ros: ros,
    name: uavname + devices_msg[uavcategory]["services"]["sincronize"]["name"],
    serviceType:
      devices_msg[uavcategory]["services"]["sincronize"]["serviceType"],
  });

  let request = new ROSLIB.ServiceRequest({});
  sincronizemessage.callService(
    request,
    function (result) {
      console.log("send threat");
      console.log(result);
      if (result.success) {
        return {
          state: "success",
          msg: "Sincronize to" + uavname + " ok" + result.message,
        }; //notification('success',"Load mission to:" + cur_roster + " ok");
      } else {
        return {
          state: "fail",
          msg: "Sincronize to:" + uavname + " fail" + result.message,
        }; //notification('danger',"Load mission to:" + cur_roster + " fail");
      }
    },
    function (result) {
      console.log("Error:" + result);
      return {
        state: "fail",
        msg: "Sincronize to:" + uavname + " Error:" + result,
      };
    }
  );
}

function loadMission(mission) {
  console.log("  load  - mission");
  let cur_ns = "",
    cur_roster = [];
  let mode_yaw = 0;
  let mode_gimbal = 0;
  let mode_trace = 0;
  let idle_vel = 1.8;
  let mode_landing = 0;
  uav_list.forEach(function prepare_wp(uav, idx, arr) {
    cur_ns = uav.name;
    cur_roster.push(cur_ns);
    if (uav.type !== "ext") {
      let wp_command = [];
      let yaw_pos = [];
      let gimbal_pos = [];
      let action_matrix = [];
      let param_matrix = [];
      Object.values(mission).forEach((route) => {
        console.log(route);
        if (route["uav"] == cur_ns) {
          console.log("route");
          console.log(route);
          Object.values(route["wp"]).forEach((item) => {
            let yaw, gimbal;
            let action_array = Array(10).fill(0);
            let param_array = Array(10).fill(0);
            let pos = new ROSLIB.Message({
              latitude: item.pos[0],
              longitude: item.pos[1],
              altitude: item.pos[2],
            });
            yaw = item.hasOwnProperty("yaw") ? item.yaw : 0;
            gimbal = item.hasOwnProperty("gimbal") ? item.gimbal : 0;
            if (item.hasOwnProperty("action")) {
              Object.keys(item.action).forEach((action_val, index, arr) => {
                found = Object.values(
                  devices_msg[uav.type]["attributes"]["mission_action"]
                ).find((element) => element.name == action_val);
                if (found) {
                  action_array[index] = found.id;
                  param_array[index] = found.param
                    ? item.action[action_val]
                    : 0;
                }
              });
            }
            wp_command.push(pos);
            gimbal_pos.push(gimbal);
            yaw_pos.push(yaw);
            action_matrix.push(action_array);
            param_matrix.push(param_array);
          });

          idle_vel = route.attributes.hasOwnProperty("idle_vel")
            ? route.attributes["idle_vel"]
            : idle_vel;
          mode_yaw = route.attributes.hasOwnProperty("mode_yaw")
            ? route.attributes["mode_yaw"]
            : mode_yaw;
          mode_gimbal = route.attributes.hasOwnProperty("mode_gimbal")
            ? route.attributes["mode_gimbal"]
            : mode_gimbal;
          mode_trace = route.attributes.hasOwnProperty("mode_trace")
            ? route.attributes["mode_trace"]
            : mode_trace;
          mode_landing = route.attributes.hasOwnProperty("mode_landing")
            ? route.attributes["mode_landing"]
            : mode_landing;

          let yaw_pos_msg = new ROSLIB.Message({ data: yaw_pos });
          let gimbal_pos_msg = new ROSLIB.Message({ data: gimbal_pos });
          let action_matrix_msg = new ROSLIB.Message({
            data: action_matrix.flat(),
          });
          let param_matrix_msg = new ROSLIB.Message({
            data: param_matrix.flat(),
          });

          let missionClient = new ROSLIB.Service({
            ros: ros,
            name:
              cur_ns +
              devices_msg[uav.type]["services"]["configureMission"]["name"],
            serviceType:
              devices_msg[uav.type]["services"]["configureMission"][
                "serviceType"
              ],
          });

          var request = new ROSLIB.ServiceRequest({
            type: "waypoint",
            waypoint: wp_command,
            radius: 0,
            maxVel: 10,
            idleVel: idle_vel,
            yaw: yaw_pos_msg,
            gimbalPitch: gimbal_pos_msg,
            yawMode: mode_yaw,
            traceMode: mode_trace,
            gimbalPitchMode: mode_gimbal,
            finishAction: mode_landing,
            commandList: action_matrix_msg,
            commandParameter: param_matrix_msg,
          });
          console.log("request");
          console.log(request);

          missionClient.callService(
            request,
            function (result) {
              console.log(
                "load mission " + missionClient.name + ": " + result.success
              );
              if (result.success) {
                return {
                  state: "success",
                  msg: "Loadding mission to" + cur_roster + " ok",
                }; //notification('success',"Load mission to:" + cur_roster + " ok");
              } else {
                return {
                  state: "fail",
                  msg: "Load mission to:" + cur_roster + " fail",
                }; //notification('danger',"Load mission to:" + cur_roster + " fail");
              }
            },
            function (result) {
              console.log("Error:" + result);
            }
          );
          console.log("loading mision to" + cur_ns);
        }
      });
    } else {
      // Si mandamos la mision individual para cada uav, hay que hacerlo por este camino, y descomentar la linea siguiente.
      //loadMissionToExt(cur_ns);
      // var info = "Loading Mission";
      // updateInfoCell(cur_ns, info);
    }
  });
  // La linea siguiente tiene en cuenta que se mandan las misiones de todos los uavs EXT a la vez. Si no fuese así, habría que comentar la linea siguiente.
  // loadMissionToExt(cur_ns);
  return { state: "success", msg: "Loadding mission to" + cur_roster }; //notification('success',"Loadding mission to: " + cur_roster);
}

function commandMission(uav_id) {
  let uav_ns = data.get_device_ns(uav_id);
  let uav_category = data.get_device_category(uav_id);
  let missionClient;
  if (uav_category !== "ext") {
    missionClient = new ROSLIB.Service({
      ros: ros,
      name: uav_ns + devices_msg[uav_category].services.commandMission.name,
      serviceType:
        devices_msg[uav_category].services.commandMission.serviceType,
    });

    let request;
    if (uav_category !== "dji_M300") {
      request = new ROSLIB.ServiceRequest({ data: true });
    } else {
      request = new ROSLIB.ServiceRequest({});
    }
    missionClient.callService(request, function (result) {
      console.log(result.message);
      if (result.success) {
        return { state: "success", msg: "Start mission:" + uav_ns + " ok" }; //notification('success',"Start mission:" + cur_roster + " ok");
      } else {
        return { state: "fail", msg: "Start mission:" + uav_ns + " FAIL!" }; //notification('danger',"Start mission:" + cur_roster + " FAIL!");
      }
    });
  } else {
    //commandMissionToEXT(uav.name);
  }
  return { state: "success", msg: "Commanding mission to: " + uav_ns }; //notification('success',"Commanding mission to: " + cur_roster);
}

function commandMissionAll() {
  var r = true;
  let alldevices = data.state.devices;
  if (r === true) {
    Object.keys(alldevices).forEach((device_id) => {
      console.log("command mission to " + device_id);
      commandMission(device_id);
    });
  } else {
    console.log("Mission canceled");
    return { state: "fail", msg: "Mission canceled" };
  }
}

async function disConnectAddUav(uav_ns) {
  let device_del = data.state.devices[uav_ns];
  if (device_del.hasOwnProperty("camera")) {
    for (let i = 0; i < device_del.camera.length; i = i + 1) {
      if (device_del.camera[i]["type"] == "WebRTC") {
        await fetch(
          `http://localhost:9997/v2/config/paths/remove/${device_del.name}_${device_del.camera[i].source}`,
          {
            method: "POST",
          }
        );
      }
    }
  }

  data.removedevice({ id: uav_ns });

  let Key_listener = Object.keys(uav_list[uav_ns]).filter((element) =>
    element.includes("listener")
  );
  let cur_uav_idx = uav_ns; //uav_list.length-1;
  console.log(Key_listener);
  if (uav_list.length != 0) {
    Key_listener.forEach((element) => {
      console.log(element);
      uav_list[cur_uav_idx][element].unsubscribe();
    });
    uav_list[cur_uav_idx] = null;
    //uav_list.slice(cur_uav_idx,1)
    console.log("Último dron eliminado de la lista");
    if (uav_list.length > 0) {
      console.log("\nLa Lista de uav actualizada es: ");
      uav_list.forEach(function (elemento, indice, array) {
        console.log(elemento, indice);
      });
    } else {
      console.log("No quedan drones en la lista");
      uav_list = [];
    }
    return { state: "success", msg: "Se ha eliminado el " + cur_uav_idx }; //notification('success',"Commanding mission to: " + cur_roster);
  } else {
    return { state: "success", msg: "no quedan UAV de la lista" };
  }
}

app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "../build", "index.html"));
});

app.get("/api/devices", (req, res) => {
  console.log("devicesget");
  res.json(Object.values(data.state.devices));
});

app.get("/api/positions", (req, res) => {
  console.log("positionsget");
  res.json(Object.values(data.state.positions));
});

app.post("/api/devices", async function (req, res) {
  console.log("devicespost");
  console.log(req.body.uav_ns);
  let myresponse = await connectAddUav(req.body);
  return res.json(myresponse);
});

app.get("/api/devices/type", async function (req, res) {
  console.log("devices type");
  return res.json(Object.keys(devices_msg));
});
app.get("/api/mission/atributes/:type", async function (req, res) {
  console.log("devices attributes " + req.params.type);
  return res.json(
    Object.values(devices_msg[req.params.type]["attributes"]["mission_param"])
  );
});
app.get("/api/mission/atributesparam/:type/:param", async function (req, res) {
  console.log("devices atributes " + req.params.type + "-" + req.params.param);
  console.log(
    devices_msg[req.params.type]["attributes"]["mission_param"][
      req.params.param
    ]["param"]
  );
  return res.json(
    devices_msg[req.params.type]["attributes"]["mission_param"][
      req.params.param
    ]["param"]
  );
});

app.get("/api/mission/actions/:type", async function (req, res) {
  console.log("devices acction " + req.params.type);
  return res.json(
    Object.values(devices_msg[req.params.type]["attributes"]["mission_action"])
  );
});

app.post("/api/loadmission", async function (req, res) {
  console.log("loadmission-post");
  //console.log(req.body.uav_ns)
  let myresponse = await loadMission(req.body.mission);
  return res.json(myresponse);
});

app.post("/api/commandmission", async function (req, res) {
  console.log("command-mission-post");
  console.log(req.body);
  let myresponse = await commandMissionAll();
  return res.json(myresponse);
});
app.post("/api/commandmission/:id", async function (req, res) {
  console.log("command-mission-post");
  console.log(req.body);
  let myresponse = await commandMission(req.params.id);
  return res.json(myresponse);
});
app.post("/api/sendTask", async function (req, res) {
  console.log("command-sendtask");
  let uav = "uav_1";
  //let home = [37.193736, -6.702947, 50];
  let home = [37.134092, -6.472401, 50];
  let reqRoute = Object.values(req.body.loc);
  let mission = {
    version: "3",
    route: [{ name: "datetime", uav: "uav_1", wp: [] }],
    status: "OK",
  };
  let response = { uav: "uav_1", points: [], status: "OK" };
  response.points.push(home);
  for (let i = 0; i < reqRoute.length; i = i + 1) {
    let wp_len = reqRoute[i].length;
    for (let j = 0; j < wp_len; j = j + 1) {
      if (j == 0) {
        response.points.push([
          reqRoute[i][j]["lat"],
          reqRoute[i][j]["lon"],
          50,
        ]);
      }
      response.points.push([reqRoute[i][j]["lat"], reqRoute[i][j]["lon"], 30]);
      if (j == +wp_len + -1) {
        response.points.push([
          reqRoute[i][j]["lat"],
          reqRoute[i][j]["lon"],
          50,
        ]);
      }
    }
  }

  response.points.push(home);
  mission.route[0]["wp"] = response.points.map((element) => {
    return { pos: element };
  });
  console.log(mission.route[0]["wp"][0]["pos"]);
  wss.clients.forEach(function each(ws) {
    ws.send(JSON.stringify({ mission: { name: "name", mission: mission } }));
  });
  let myresponse = { response };
  return res.json(myresponse);
});
//app.post('/api/threat',async function(req,res){
//  console.log('threat-post')
//console.log(req.body.uav_ns)
//  let myresponse = await threatUAV(req.body.uav_ns);
//  return res.json(myresponse);
//});
app.post("/api/commands", async function (req, res) {
  //here get id and description, where description is string like threat,1 or sincronize, landing,1
  console.log("command");
  console.log(req.body);
  let response = {
    state: "fail",
    msg: "Command to:" + data.get_device_ns(req.body.uav_id) + " no exist",
  };
  if (req.body.description == "threat") {
    response = await threatUAV(req.body.uav_id);
  } else if (req.body.description == "sincronize") {
    response = await sincronizeUAV(req.body.uav_id);
  }
  return res.json(response);
});

app.post("/api/disconectdevice", async function (req, res) {
  console.log("loadmission-post");
  //console.log(req.body.uav_ns)
  let myresponse = await disConnectAddUav(req.body.uav_ns);
  return res.json(myresponse);
});

app.post("/api/rosConnect", async function (req, res) {
  console.log("rosconect");
  rosConnect();
  await sleep(200);
  return res.json(rosState);
});

app.get("/api/rosConnect", function (req, res) {
  console.log("getrosconnect");
  return res.json(rosState);
});

app.get("/api/topics", async function (req, res) {
  console.log("rosTopic");
  const topiclist = await getTopics2();
  return res.json(topiclist);
});

app.get("/api/test.png", (req, res) => {
  // A 1x1 pixel red colored PNG file.
  const img = Buffer.from(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8DwHwAFBQIAX8jx0gAAAABJRU5ErkJggg==",
    "base64"
  );
  res.writeHead(200, {
    "Content-Type": "image/png",
    "Content-Length": img.length,
  });
  res.end(img);
});
// recibir un  array de objetos  y devolver el mismo array con la longitud y la altitud
app.get("/api/map/elevation", async function (req, res) {
  console.log("using ---- elevation");
  let locations = req.query.locations;
  let myresponse = await ApiElevation(locations);
  res.json(myresponse);
});

function divideLineIntoPoints(line, steps, dist) {
  console.log("line");
  console.log(line);
  let dividedLine = []; // Start with the first point
  let accumulatedDistance = 0;
  let prevPoint = line[0];
  let currentPoint = line[1];
  let latStep = (currentPoint[0] - prevPoint[0]) / steps;
  let lngStep = (currentPoint[1] - prevPoint[1]) / steps;
  let distStep = dist / steps;
  for (let i = 1; i < steps; i++) {
    let newLat = prevPoint[0] + i * latStep;
    let newLon = prevPoint[1] + i * lngStep;
    let newdist = i * distStep;
    dividedLine.push({ lat: newLat, lon: newLon, dist: newdist });
  }

  //dividedLine.push(line[line.length - 1]); // Add the last point
  console.log("dividedLine------------");
  console.log(dividedLine);
  return dividedLine;
}
// recibir un  array de objetos  y devolver el mismo array con la longitud y la altitud
app.post("/api/map/elevation", async function (req, res) {
  console.log("using ---- elevation");
  listpoint = req.body.routes;
  console.log(listpoint);
  let wpaltitude = [];
  let listwaypoint = [];
  if (listpoint.length > 0) {
    listpoint.forEach((route, index_rt, array_rt) => {
      let acumulative = [];
      let lastindex = 0;
      let lastdist = 0;
      wpaltitude.push([]);
      route.forEach((wp, index, array) => {
        let lineLength = 0;
        let altitud = wp[2];
        acumulative.push(wp);
        if (index != 0) {
          let linestring = turf.lineString(acumulative);
          lineLength = turf.length(linestring, { units: "meters" });
          //medir que distancia sea mayor
          let otherline = turf.lineString([wp, array[lastindex]]);
          let distbetweenwp = turf.length(otherline, { units: "meters" });
          console.log("distancia puntos " + distbetweenwp);
          if (distbetweenwp > 200) {
            console.log("mayor a 200 metros");
            //funcion de slice and add to  //altitud = -1;
            let steps = Math.floor(distbetweenwp / 200) + 1;
            let newpoints = divideLineIntoPoints(
              [array[lastindex], wp],
              steps,
              distbetweenwp
            );
            newpoints.map((nwp) => {
              listwaypoint.push([nwp.lat, nwp.lon]);
              let nwpdist = +lastdist.toFixed(1) + +nwp.dist.toFixed(1);
              console.log(
                "lastdist " + lastdist + " caldist " + nwp.dist.toFixed(1)
              );

              wpaltitude[index_rt].push({
                length: nwpdist,
                uav: null,
              });
            });
          }
        }
        listwaypoint.push([wp[0], wp[1]]);
        wpaltitude[index_rt].push({
          length: Number(lineLength.toFixed(1)),
          uav: altitud,
        });
        lastindex = index;
        lastdist = Number(lineLength.toFixed(1));
      });
    });
    console.log(wpaltitude);
    console.log(listwaypoint);

    let elevationprofile = await ApiElevation(listwaypoint);
    //anadir elevacion profile
    console.log(elevationprofile);
    let auxcount = 0;
    let initElevationIndex = 0;
    for (let index_rt = 0; index_rt < wpaltitude.length; index_rt++) {
      let wp_count = 0;
      for (let index = 0; index < wpaltitude[index_rt].length; index++) {
        wpaltitude[index_rt][index]["elevation"] = Number(
          elevationprofile.results[auxcount].elevation.toFixed(1)
        );
        wpaltitude[index_rt][index]["lat"] = Number(
          elevationprofile.results[auxcount].location.lat
        );
        wpaltitude[index_rt][index]["lng"] = Number(
          elevationprofile.results[auxcount].location.lng
        );

        if (index == 0) {
          initElevationIndex = auxcount;
        }

        wpaltitude[index_rt][index]["rt"] = index_rt;
        if (wpaltitude[index_rt][index]["uav"] !== null) {
          wpaltitude[index_rt][index]["wp"] = wp_count;
          wpaltitude[index_rt][index]["uavheight"] = (
            +wpaltitude[index_rt][index]["uav"] +
            +elevationprofile.results[initElevationIndex].elevation
          ).toFixed(1);
          wp_count = wp_count + 1;
        }
        auxcount = auxcount + 1;
      }
    }
  }
  console.log(wpaltitude);
  res.json(wpaltitude);
});

const ApiElevation = async (LocationList) => {
  let myresponse = {};
  let divLocationList = [];
  let maxAPIlocation = 99;
  if (LocationList.length > maxAPIlocation) {
  }
  for (let i = 0; i < LocationList.length; i = i + maxAPIlocation) {
    if (LocationList.length - i < maxAPIlocation) {
      divLocationList.push(LocationList.slice(i));
      break;
    }
    divLocationList.push(LocationList.slice(i, i + maxAPIlocation));
  }
  divLocationList.map((element) => console.log(element.length));

  let stringLocationList = divLocationList.map((list) =>
    list.map((waypoint) => waypoint.join(",")).join("|")
  );
  divLocationList.map((element) => console.log(element));
  //`https://api.opentopodata.org/v1/eudem25m?locations=${stringLocationList}`
  //`https://maps.googleapis.com/maps/api/elevation/json?locations=${stringLocationList}&key=AIzaSyBglf9crAofRVtqTqfz7ZpdATsZY_H3ZFE`
  console.log("response elevation----");
  for (let i = 0; i < stringLocationList.length; i = i + 1) {
    await fetch(
      `https://api.opentopodata.org/v1/eudem25m?locations=${stringLocationList[i]}`
    )
      .then((response) => response.json())
      .then((body) => {
        console.log(body);
        if (myresponse.hasOwnProperty("results")) {
          body.results.map((element) => {
            myresponse.results.push(element);
          });
        } else {
          myresponse = body;
        }
      });
  }

  return myresponse;
};
app.get("/api/placeholder", (req, res) => {
  // A 1x1 pixel red colored PNG file.
  const base64 = fs.readFileSync("../src/assets/img/placeholder.jpg", "base64");
  //console.log(base64)
  const img = Buffer.from(base64, "base64");
  //console.log(img)
  res.writeHead(200, {
    "Content-Type": "image/png",
    "Content-Length": img.length,
  });
  res.end(img);
});

app.delete("/api/:endpoint/:itemid", async (req, res) => {
  console.log("Delete UAV" + req.params.itemid);
  console.log(req.params);
  //av_list[req.params.itemid].listener.unsubscribe();
  let myresponse = await disConnectAddUav(req.params.itemid);
  console.log(uav_list);
  res.send(req.params);
});

// camera
app.get("/api/media/:deviceid", (req, res) => {
  console.log("cameraget---");
  console.log("cameraget" + req.params.deviceid);
  if (data.state.camera[req.params.deviceid]) {
    let base64 = data.state.camera[req.params.deviceid].camera;
    res.type("image/png");
    res.send(base64);
  } else {
    res.status(404).end();
  }
});
// camera
app.get("/api/media1/:deviceid", (req, res) => {
  console.log("cameraget1---");
  console.log("cameraget --" + req.params.deviceid);
  if (data.state.camera[req.params.deviceid]) {
    let base64 = data.state.camera[req.params.deviceid].camera;
    //console.log(base64)
    const img = Buffer.from(base64, "base64");
    //console.log(img)
    res.writeHead(200, {
      "Content-Type": "image/png",
      "Content-Length": img.length,
    });
    res.end(img);
  } else {
    res.status(404).end();
  }
});
