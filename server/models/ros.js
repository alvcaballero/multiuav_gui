import ROSLIB from 'roslib';
import { readYAML, getDatetime } from '../common/utils.js';
import { DevicesController } from '../controllers/devices.js';
import { missionSMModel } from './missionSM.js';
import { positionsController } from '../controllers/positions.js';
import { categoryController } from '../controllers/category.js';

var ros = '';
const rosState = { state: 'disconnect', msg: 'init msg' };
const devices_msg = readYAML('../config/devices/devices_msg.yaml');
const service_list = [];
const uav_list = [];

var autoconectRos = setInterval(() => {
  console.log('autoconectRos');
  if (rosState.state != 'connect') {
    rosModel.rosConnect();
  }
}, 30000);

console.log('Load model ROS');
export class rosModel {
  static setrosState({ state, msg }) {
    rosState['state'] = state;
    rosState['msg'] = msg;
  }
  static serverStatus() {
    return rosState;
  }

  static async connectAllUAV() {
    const devices = await DevicesController.getAllDevices();
    for (let device of Object.values(devices)) {
      await rosModel.subscribeDevice({
        id: device.id,
        name: device.name,
        type: device.category,
        camera: device.camera,
        watch_bound: true,
        bag: false,
      });
    }
    console.log('finish to connect all devices ------------');
  }

  static async rosConnect() {
    console.log('try to connect to ros');
    if (rosState.state != 'connect') {
      ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
      ros.on('connection', function () {
        console.log('ROS Connected to websocket server.');
        rosModel.setrosState({ state: 'connect', msg: 'Conectado a ROS' });
        rosModel.connectAllUAV();
        rosModel.GCSServicesMission();
        clearInterval(autoconectRos);
      });
      ros.on('error', function (error) {
        console.log('ROS Error connecting to websocket server: ', error);
        rosModel.setrosState({ state: 'error', msg: 'No se ha posido conectar a ROS' });
        autoconectRos = setInterval(() => {
          if (rosState.state != 'connect') {
            rosModel.rosConnect();
          }
        }, 30000);
      });
      ros.on('close', function () {
        console.log('ROS Connection to websocket server closed.');
        rosModel.setrosState({ state: 'disconnect', msg: 'Desconectado a ROS' });
        autoconectRos = setInterval(() => {
          if (rosState.state != 'connect') {
            rosModel.rosConnect();
          }
        }, 30000);
      });
    } else {
      rosModel.unsubscribe(-1);
      ros.close();
    }
  }

  static decodeMsg(uav_id, uav_type, type, msgType) {
    if (type == 'position' && msgType == 'sensor_msgs/NavSatFix') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          id: msg.header.seq,
          deviceId: uav_id,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: getDatetime(), // "2023-03-09T22:12:44.000+00:00",
        });
      });
    }
    if (type == 'sensor_height' && msgType == 'mavros_msgs/Altitude') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        //var showData = document.getElementById(uav_ns).cells;
        //showData[1].innerHTML = (message.relative).toFixed(2);
      });
    }
    if (type == 'sensor_height' && msgType == 'std_msgs/Float32') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        //Altitud de ultrasonico
        //positionsController.updatePosition({deviceId:uav_id,altitude:msg.data});
      });
    }
    if (type == 'vo_position' && msgType == 'dji_osdk_ros/VOPosition') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        //dispatch(dataActions.updatePosition({id:msg.header.seq,deviceId:uav_id,latitude:msg.x,longitude:msg.y,altitude:msg.z,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"}));
      });
    }
    if (type == 'IMU' && msgType == 'sensor_msgs/Imu') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        //https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        let q = msg.orientation;
        let yaw = Math.atan2(2.0 * (q.x * q.y + q.w * q.z), -1 + 2 * (q.w * q.w + q.x * q.x)) * -1;
        positionsController.updatePosition({ deviceId: uav_id, course: 90 + yaw * 57.295 });
      });
    }
    if (type == 'hdg' && msgType == 'std_msgs/Float64') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({ deviceId: uav_id, course: msg.data }); //uav_list[uav_id].marker.setRotationAngle(message.data)
      });
    }

    if (
      type == 'speed' &&
      msgType == 'geometry_msgs/Vector3Stamped' &&
      uav_type.includes('dji_M300')
    ) {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          deviceId: uav_id,
          speed: msg.velocity * 0.01,
        });
      });
      return null;
    }

    if (type == 'speed' && msgType == 'geometry_msgs/Vector3Stamped') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          deviceId: uav_id,
          speed: Math.sqrt(Math.pow(msg.vector.x, 2) + Math.pow(msg.vector.y, 2)).toFixed(2),
        });
      });
    }
    if (type == 'speed' && msgType == 'geometry_msgs/TwistStamped') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          deviceId: uav_id,
          speed: Math.sqrt(
            Math.pow(msg.twist.linear.x, 2) + Math.pow(msg.twist.linear.y, 2)
          ).toFixed(2),
        }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
      });
    }
    if (type == 'battery' && uav_type == 'px4') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          deviceId: uav_id,
          batteryLevel: (msg.percentage * 100).toFixed(0),
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });
      return null;
    }
    if (type == 'battery') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({ deviceId: uav_id, batteryLevel: msg.percentage });
      });
    }
    if (type == 'gimbal' && msgType == 'geometry_msgs/Vector3Stamped') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({ deviceId: uav_id, gimbal: msg.vector }); // showData[3].innerHTML = message.percentage + "%";
      });
    }
    if (type == 'obstacle_info' && msgType == 'dji_osdk_ros/ObstacleInfo') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({ deviceId: uav_id, obstacle_info: msg }); // showData[3].innerHTML = message.percentage + "%";
      });
    }
    if (type == 'camera' && msgType == 'sensor_msgs/CompressedImage') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updateCamera({ deviceId: uav_id, camera: msg.data }); //positionsController.updateCamera({deviceId:uav_id,camera:"data:image/jpg;base64," + msg.data});//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
      });
    }
    if (type == 'flight_status' && msgType == 'std_msgs/UInt8') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          deviceId: uav_id,
          protocol: 'dji',
          landed_state: msg.data,
        });
      });
    }
    if (type == 'threat' && msgType == 'std_msgs/Bool') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({ deviceId: uav_id, threat: msg.data });
      });
    }
    if (type == 'state_machine' && msgType == 'std_msgs/String') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          deviceId: uav_id,
          protocol: 'catec',
          mission_state: '0',
          wp_reached: '0',
          uav_state: 'ok',
          landed_state: msg.data,
        });
      });
    }
    if (type == 'state_machine' && msgType == 'muav_state_machine/UAVState') {
      uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
        positionsController.updatePosition({
          deviceId: uav_id,
          protocol: msg.airframe_type,
          mission_state: msg.mission_state,
          wp_reached: msg.wp_reached,
          uav_state: msg.uav_state,
          landed_state: msg.landed_state,
        });
      });
    }
  }

  static async decodeMissionMsg({ uav_id, route }) {
    let device = await DevicesController.getDevice(uav_id);
    //console.log(device);
    let uavname = device.name;
    let uavcategory = device.category;
    let mode_yaw = 0;
    let mode_gimbal = 0;
    let mode_trace = 0;
    let idle_vel = 1.8;
    let max_vel = 10;
    let mode_landing = 0;
    let response = null;
    let wp_command = [];
    let yaw_pos = [];
    let speed_pos = [];
    let gimbal_pos = [];
    let action_matrix = [];
    let param_matrix = [];
    if (route['uav'] == uavname) {
      //console.log('route'); //console.log(route);
      idle_vel = route.attributes.hasOwnProperty('idle_vel')
        ? route.attributes['idle_vel']
        : idle_vel;
      max_vel = route.attributes.hasOwnProperty('max_vel') ? route.attributes['max_vel'] : max_vel;
      mode_yaw = route.attributes.hasOwnProperty('mode_yaw')
        ? route.attributes['mode_yaw']
        : mode_yaw;
      mode_gimbal = route.attributes.hasOwnProperty('mode_gimbal')
        ? route.attributes['mode_gimbal']
        : mode_gimbal;
      mode_trace = route.attributes.hasOwnProperty('mode_trace')
        ? route.attributes['mode_trace']
        : mode_trace;
      mode_landing = route.attributes.hasOwnProperty('mode_landing')
        ? route.attributes['mode_landing']
        : mode_landing;

      let categoryModel = categoryController.getActionsParam({ type: uavcategory });

      Object.values(route['wp']).forEach((item) => {
        let yaw, gimbal, speed;
        let action_array = Array(10).fill(0);
        let param_array = Array(10).fill(0);
        let pos = new ROSLIB.Message({
          latitude: item.pos[0],
          longitude: item.pos[1],
          altitude: item.pos[2],
        });
        yaw = item.hasOwnProperty('yaw') ? item.yaw : 0;
        speed = item.hasOwnProperty('speed') ? item.speed : idle_vel;
        gimbal = item.hasOwnProperty('gimbal') ? item.gimbal : 0;
        if (item.hasOwnProperty('action')) {
          Object.keys(item.action).forEach((action_val, index, arr) => {
            let found = Object.values(categoryModel).find((element) => element.name == action_val);
            if (found) {
              action_array[index] = Number(found.id);
              param_array[index] = found.param ? Number(item.action[action_val]) : 0;
            }
          });
        }
        wp_command.push(pos);
        gimbal_pos.push(gimbal);
        yaw_pos.push(yaw);
        speed_pos.push(speed);
        action_matrix.push(action_array);
        param_matrix.push(param_array);
      });

      let yaw_pos_msg = new ROSLIB.Message({ data: yaw_pos });
      let speed_pos_msg = new ROSLIB.Message({ data: speed_pos });
      let gimbal_pos_msg = new ROSLIB.Message({ data: gimbal_pos });
      let action_matrix_msg = new ROSLIB.Message({
        data: action_matrix.flat(),
      });
      let param_matrix_msg = new ROSLIB.Message({
        data: param_matrix.flat(),
      });

      response = {
        type: 'waypoint',
        waypoint: wp_command,
        radius: 0,
        maxVel: max_vel,
        idleVel: idle_vel,
        yaw: yaw_pos_msg,
        speed: speed_pos_msg,
        gimbalPitch: gimbal_pos_msg,
        yawMode: mode_yaw,
        traceMode: mode_trace,
        gimbalPitchMode: mode_gimbal,
        finishAction: mode_landing,
        commandList: action_matrix_msg,
        commandParameter: param_matrix_msg,
      };
    }

    return response;
  }

  // Subscribing
  // create subcribin mesage
  static async subscribeDevice(uavAdded) {
    uav_list.push(uavAdded);
    //console.log(uavAdded);
    let uav_type = uavAdded.type;
    let cur_uav_idx = uavAdded.id;
    let uav_ns = uavAdded.name;
    let uav_camera = uavAdded.camera;

    Object.keys(devices_msg[uav_type]['topics']).forEach((element) => {
      uav_list[cur_uav_idx]['listener_' + element] = new ROSLIB.Topic({
        ros: ros,
        name: uav_ns + devices_msg[uav_type]['topics'][element]['name'], //'/dji_osdk_ros/rtk_position',
        messageType: devices_msg[uav_type]['topics'][element]['messageType'],
      });
    });

    Object.keys(devices_msg[uav_type]['topics']).forEach((element) => {
      if (element !== 'camera') {
        this.decodeMsg(
          cur_uav_idx,
          uav_type,
          element,
          devices_msg[uav_type]['topics'][element]['messageType']
        );
      }
    });

    for (let i = 0; i < uav_camera.length; i = i + 1) {
      if (uav_camera[i]['type'] == 'Websocket') {
        this.decodeMsg(
          cur_uav_idx,
          uav_type,
          'camera',
          devices_msg[uav_type]['topics']['camera']['messageType']
        );
      }
    }
  }

  static async unsubscribeDevice(id) {
    console.log('unsuscribe model');
    let cur_uav_idx;
    let Key_listener;
    if (uav_list.length != 0) {
      if (id < 0) {
        console.log('unsuscribe all');
        for (let i = 0; i < uav_list.length; i++) {
          //uav_list[i].listener.unsubscribe();
          cur_uav_idx = uav_list.findIndex((element) => element.id == id);
          if (cur_uav_idx) {
            Key_listener = Object.keys(uav_list[i]).filter((element) =>
              element.includes('listener')
            );
            Key_listener.forEach((element) => {
              uav_list[i][element].unsubscribe();
            });
          }
        }
        uav_list = [];
      } else {
        console.log('unsuscribe ' + id);
        cur_uav_idx = uav_list.findIndex((element) => element.id == id);

        Key_listener = Object.keys(uav_list[cur_uav_idx]).filter((element) =>
          element.includes('listener')
        );

        console.log(Key_listener);
        if (uav_list.length != 0) {
          Key_listener.forEach((element) => {
            console.log(element);
            uav_list[cur_uav_idx][element].unsubscribe();
          });
          delete uav_list[cur_uav_idx];

          //uav_list.slice(cur_uav_idx,1)
          console.log('Último dron eliminado de la lista');
          if (uav_list.length > 0) {
            console.log('\nLa Lista de uav actualizada es: ');
            uav_list.forEach(function (elemento, indice, array) {
              console.log(elemento, indice);
            });
          } else {
            console.log('No quedan drones en la lista');
            uav_list = [];
          }
          return { state: 'success', msg: 'Se ha eliminado el ' + cur_uav_idx }; //notification('success',"Commanding mission to: " + cur_roster);
        }
      }
    } else {
      return { state: 'success', msg: 'no quedan UAV de la lista' };
    }
  }

  static async callService({ uav_id, type, request }) {
    if (rosState.state != 'connect') {
      return { state: 'error', msg: 'ROS no conectado' };
    }
    let device = await DevicesController.getDevice(uav_id);
    let uavName = device.name;
    let uavCategory = device.category;
    console.log(device);

    if (!devices_msg[uavCategory]['services'].hasOwnProperty(type)) {
      console.log(type + ' to:' + uavName + ' dont have this service');
      return { state: 'warning', msg: type + ' to:' + uavName + ' dont have this service' };
    }

    let Message = new ROSLIB.Service({
      ros: ros,
      name: uavName + devices_msg[uavCategory]['services'][type]['name'],
      serviceType: devices_msg[uavCategory]['services'][type]['serviceType'],
    });

    let MsgRequest;
    // &&;
    if (devices_msg[uavCategory]['services'][type]['serviceType'] == 'std_srvs/TriggerRequest') {
      MsgRequest = new ROSLIB.ServiceRequest({});
    } else {
      if (request) {
        MsgRequest = new ROSLIB.ServiceRequest(request);
      } else {
        MsgRequest = new ROSLIB.ServiceRequest({});
      }
    }

    return new Promise((resolve, rejects) => {
      Message.callService(
        MsgRequest,
        function (result) {
          console.log('send command  ' + type + 'to uav_id' + uav_id);
          console.log(result);
          if (result.success || result.result) {
            resolve({
              state: 'success',
              msg: type + ' to' + uavName + ' ok',
            });
          } else {
            resolve({
              state: 'error',
              msg: type + ' to:' + uavName + ' error',
            });
          }
        },
        function (result) {
          console.log('Error:' + result);
          resolve({ state: 'error', msg: 'Error:' + result });
        }
      );
    });
  }

  static getTopics() {
    var topicsClient = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/topics',
      serviceType: 'rosapi/Topics',
    });

    var request = new ROSLIB.ServiceRequest();
    return new Promise((resolve, rejects) => {
      topicsClient.callService(request, function (result) {
        resolve(result.topics);
      });
    });
  }
  static getTopics2() {}

  /*
   / Create Ros services that UAV can consume for 
   / Check if UAV finish mission
   / Check if UAV finish Download
  */
  static GCSServicesMission() {
    let cur_uav_idx = String(service_list.length);

    service_list.push({ name: 'service mission' });
    service_list[cur_uav_idx]['ServiceMission'] = new ROSLIB.Service({
      ros: ros,
      name: '/GCS/FinishMission',
      serviceType: 'aerialcore_common/finishMission',
    });

    service_list[cur_uav_idx]['ServiceMission'].advertise(function (request, response) {
      // Call state machine
      console.log('callback Sevice finish mission');
      console.log(request);
      if (request.hasOwnProperty('uav_id')) {
        let mydevice = DevicesController.getByName(`uav_${request.uav_id}`);
        console.log(mydevice);
        console.log('mydevice in finish mission ' + mydevice.id);
        missionSMModel.UAVFinishMission(mydevice.id);
      }

      response['success'] = true;
      response['msg'] = 'Set successfully';
      return true;
    });
    console.log('ServiceDownload');

    cur_uav_idx = String(service_list.length);
    service_list.push({ name: 'ServiceDownload' });

    service_list[cur_uav_idx]['ServiceDownload'] = new ROSLIB.Service({
      ros: ros,
      name: '/GCS/FinishDownload',
      serviceType: 'aerialcore_common/finishGetFiles',
    });

    service_list[cur_uav_idx]['ServiceDownload'].advertise(function (request, response) {
      console.log('callback Service finish download files');
      console.log(request);
      if (request.hasOwnProperty('uav_id')) {
        let mydevice = DevicesController.getByName(request.uav_id);
        console.log(mydevice);
        console.log('mydevice finish download files' + mydevice.id);
        missionSMModel.DownloadFiles(mydevice.id);
      }
      response['success'] = true;
      response['msg'] = 'Set successfully';
      return true;
    });
  }
  static getServices() {
    const servicesPromise = new Promise((resolve, reject) => {
      ros.ROS.getServices(
        (services) => {
          const serviceList = services.map((serviceName) => {
            return {
              serviceName,
            };
          });
          resolve({
            services: serviceList,
          });
          reject({
            services: [],
          });
        },
        (message) => {
          console.error('Failed to get services', message);
          ros.services = [];
        }
      );
    });
    servicesPromise.then((services) => setROS((ros) => ({ ...ros, services: services })));
    return ros.services;
  }

  static Getservicehost(nameService) {
    let servicehost = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/service_host',
      serviceType: 'rosapi/ServiceHost',
    });

    let request = new ROSLIB.ServiceRequest({ service: nameService });

    return new Promise((resolve, rejects) => {
      servicehost.callService(request, function (result) {
        resolve(result.host);
      });
    });
  }

  async getListMaster() {
    let servicemaster = new ROSLIB.Service({
      ros: ros,
      name: '/master_discovery/list_masters',
      serviceType: 'multimaster_msgs_fkie/DiscoverMasters',
    });

    let request = new ROSLIB.ServiceRequest();

    return new Promise((resolve, rejects) => {
      servicemaster.callService(request, function (result) {
        console.log('masterip -- ' + result.length);
        resolve(result);
      });
    });
  }
}
