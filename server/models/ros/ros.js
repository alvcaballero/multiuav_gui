import * as ROSLIB from 'roslib';
import { readDataFile, getDatetime } from '../../common/utils.js';
import { devicesController } from '../../controllers/devices.js';
import { missionController } from '../../controllers/mission.js';
import { positionsController } from '../../controllers/positions.js';
import { decodeRosMsg } from './rosDecode.js';
import { encodeRosSrv } from './rosEncode.js';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { categoryController } from '../../controllers/category.js';
import logger, { logHelpers } from '../../common/logger.js';
import { ROS2GoalActionClient } from './rosActionClient.js';
import { error } from 'console';

var ros = null;
const rosState = { state: 'disconnect', msg: 'init msg' };
const devices_msg = readDataFile('../config/devices/devices_msg.yaml');
const service_list = [];
const uav_list = {};

var autoconectRos = null;
var noTimerflag = true;
logHelpers.system.info('roslib version:', ROSLIB.version || 'unknown');

function connectRos() {
  if (rosState.state != 'connect') {
    rosModel.rosConnect();
  } else {
    clearInterval(autoconectRos);
    noTimerflag = true;
  }
}
function autoConectRos() {
  if (noTimerflag) {
    noTimerflag = false;
    autoconectRos = setInterval(connectRos, 30000);
  }
}

export class rosModel {
  static setrosState({ state, msg }) {
    rosState['state'] = state;
    rosState['msg'] = msg;
  }
  static serverStatus() {
    return rosState;
  }

  static async connectAllUAV() {
    const devices = await devicesController.getAllDevices();
    //console.log('begin to conncet all devices ------------');
    for (let device of Object.values(devices)) {
      if (device.protocol == 'ros') {
        await rosModel.subscribeDevice({
          id: device.id,
          name: device.name,
          category: device.category,
          camera: device.camera,
          watch_bound: true,
          bag: false,
        });
      }
    }
    //console.log('finish to connect all devices ------------');
  }

  static disconectRos() {
    rosModel.unsubscribeDevice(-1);
    rosModel.GCSunServicesMission();
    autoConectRos();
  }

  static async rosConnect() {
    if (rosState.state != 'connect') {
      ros = new ROSLIB.Ros({ url: 'ws://127.0.0.1:9090', encoding: 'utf8' });
      ros.on('connection', function () {
        logHelpers.ros.connect('server', { status: 'connected' });
        rosModel.setrosState({ state: 'connect', msg: 'Conectado a ROS' });
        rosModel.connectAllUAV();
        rosModel.GCSServicesMission();
      });
      ros.on('error', function (error) {
        logHelpers.ros.error('Error connect to server', error);
        rosModel.setrosState({ state: 'error', msg: 'No se ha podido conectar a ROS' });
        rosModel.disconectRos();
      });
      ros.on('close', function () {
        logHelpers.ros.error('Connection closed.', { message: 'Connection closed' });
        rosModel.setrosState({ state: 'disconnect', msg: 'Desconectado a ROS' });
        rosModel.disconectRos();
      });
    }
  }

  // Subscribing
  static RosSubscribe(uav_id, uav_type, type, msgType, callback) {
    uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
      positionsController.updatePosition(callback({ msg, deviceId: uav_id, uav_type, type, msgType }));
    });
  }
  static RosSubscribeCamera(uav_id, uav_type, type, msgType, callback) {
    uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
      positionsController.updateCamera(callback({ msg, deviceId: uav_id, uav_type, type, msgType }));
    });
  }
  // Proccess Ros for all devices
  static async subscribeDevice(uavAdded) {
    if (rosState.state != 'connect') {
      return { state: 'error', msg: 'ROS no conectado' };
    }
    logHelpers.ros.subscribe(uavAdded.id, uavAdded.name, { category: uavAdded.category });

    const { id, name, category, camera } = uavAdded;
    uav_list[id] = uavAdded;
    let msgType = devices_msg[category]['topics'];
    // create listener
    Object.keys(devices_msg[category]['topics']).forEach((element) => {
      uav_list[id]['listener_' + element] = new ROSLIB.Topic({
        ros: ros,
        name: name + devices_msg[category]['topics'][element]['name'], //'/dji_osdk_ros/rtk_position',
        messageType: devices_msg[category]['topics'][element]['messageType'],
      });
    });
    // subscribe devices
    Object.keys(devices_msg[category]['topics']).forEach((element) => {
      if (element !== 'camera') {
        this.RosSubscribe(id, category, element, msgType[element]['messageType'], decodeRosMsg);
      }
    });
    // subscribe camera
    for (let i = 0; i < camera.length; i = i + 1) {
      console.log(camera[i]['type']);
      if (camera[i]['type'] == 'Websocket') {
        console.log(`camera websocket for ${name} `);
        this.RosSubscribeCamera(id, category, 'camera', msgType['camera']['messageType'], decodeRosMsg);
      }
    }
  }

  static async unsubscribeDevice(id) {
    //console.log('unsubscribe model');
    let cur_uav_idx;
    let Key_listener;
    if (Object.keys(uav_list).length != 0) {
      if (id < 0) {
        //console.log('unsubscribe all');
        for (let i = 0; i < Object.keys(uav_list).length; i++) {
          //uav_list[i].listener.unsubscribe();
          cur_uav_idx = Object.values(uav_list).find((element) => element.id == id);
          if (cur_uav_idx) {
            Key_listener = cur_uav_idx.filter((element) => element.includes('listener'));
            Key_listener.forEach((element) => {
              uav_list[cur_uav_idx.id][element].unsubscribe();
            });
          }
        }
        // uav_list = [];
        var props = Object.getOwnPropertyNames(uav_list);
        for (var i = 0; i < props.length; i++) {
          delete uav_list[props[i]];
        }
      } else {
        // console.log('unsubscribe ' + id);
        cur_uav_idx = Object.values(uav_list).find((element) => element.id == id);

        Key_listener = Object.keys(cur_uav_idx).filter((element) => element.includes('listener'));

        // console.log(Key_listener);
        if (Object.keys(uav_list).length != 0) {
          Key_listener.forEach((element) => {
            // console.log(element);
            uav_list[cur_uav_idx.id][element].unsubscribe();
          });
          delete uav_list[cur_uav_idx.id];
          return { state: 'success', msg: 'Se ha eliminado el ' + cur_uav_idx }; //notification('success',"Commanding mission to: " + cur_roster);
        }
      }
    } else {
      return { state: 'success', msg: 'no quedan UAV de la lista' };
    }
  }

  static async callRosService({ service, messageType, message }) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    const services = await rosModel.getServices();
    if (!services.includes(service)) {
      console.log('Available services:', services);
      throw new Error(`Service '${service}' not available`);
    }

    const topicType = await rosModel.getServicesType(service);
    const auxtopicType = topicType.replace('/srv/', '/');
    if (!(topicType == messageType || auxtopicType == messageType)) {
      throw new Error(`Service type mismatch: expected '${topicType}', got '${messageType}'`);
    }

    const responseSrvStructure = await rosModel.getServiceRequestDetails(messageType);
    const srvStructure = responseSrvStructure.typedefs || [];

    if (!srvStructure || srvStructure.length === 0) {
      throw new Error(`No structure found for service type '${messageType}'`);
    }

    const typeMap = buildTypeMap(srvStructure);

    validateRosMsg(messageType, message, typeMap);

    let Message = new ROSLIB.Service({
      ros: ros,
      name: service,
      serviceType: messageType,
    });

    let MsgRequest = message ? message : {};

    return new Promise((resolve, rejects) => {
      Message.callService(
        MsgRequest,
        (result) => resolve(result),
        (error) => rejects(error)
      );
    });
  }

  static async callService({ uav_id, type, request }) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    let device = await devicesController.getDevice(uav_id);
    const { name, category } = device;
    //console.log(device);

    if (!devices_msg[category]['services'].hasOwnProperty(type)) {
      //console.log(type + ' to:' + name + ' dont have this service');
      return { state: 'warning', msg: type + ' to:' + name + ' dont have this service' };
    }

    let msgType = devices_msg[category]['services'][type]['serviceType'];
    let myRequest = encodeRosSrv({ type, msg: request, msgType: msgType });
    const service = `/${name}${devices_msg[category]['services'][type]['name']}`;
    try {
      const response = await rosModel.callRosService({ service, messageType: msgType, message: myRequest });
      if (response.success || response.result) {
        return { state: 'success', msg: type + ' to' + name + ' ok' };
      } else {
        return { state: 'error', msg: type + ' to:' + name + ' error' };
      }
    } catch (error) {
      console.error('Error calling service:', error);
      return { state: 'error', msg: 'Failed to call service: ' + error.message };
    }
  }

  static getTopics() {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    return new Promise((resolve, reject) => {
      ros.getTopics(
        (topics) => resolve(topics),
        (error) => reject(error)
      );
    });
  }
  static getServices() {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    return new Promise((resolve, reject) => {
      ros.getServices(
        (services) => resolve(services),
        (error) => reject(error)
      );
    });
  }
  static async getServicesType(service) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    console.log('get service type for ' + service);
    return new Promise((resolve, reject) => {
      ros.getServiceType(
        service,
        (serviceType) => resolve(serviceType),
        (error) => reject(error)
      );
    });
  }
  static async getServiceRequestDetails(type) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    return new Promise((resolve, reject) => {
      ros.getServiceRequestDetails(
        type,
        (serviceDetails) => {
          if (!serviceDetails || serviceDetails.length === 0) {
            reject(new Error('Service type not found'));
          }
          resolve(serviceDetails);
        },
        (error) => reject(error)
      );
    });
  }
  static async getServiceResponseDetails(type) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    return new Promise((resolve, reject) => {
      ros.getServiceResponseDetails(
        type,
        (serviceDetails) => {
          if (!serviceDetails || serviceDetails.length === 0) {
            reject(new Error('Service type not found'));
          }
          resolve(serviceDetails);
        },
        (error) => reject(error)
      );
    });
  }
  static getTopicType(topic) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    console.log('get topic type for ' + topic);
    return new Promise((resolve, reject) => {
      ros.getTopicType(
        topic,
        (topicType) => resolve(topicType),
        (error) => reject(error)
      );
    });
  }
  static getMessageDetails(message) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    return new Promise((resolve, reject) => {
      ros.getMessageDetails(
        message,
        (messageDetails) => {
          if (!messageDetails || (Array.isArray(messageDetails) && messageDetails.length === 0)) {
            reject(new Error('Message type not found'));
          }
          resolve(messageDetails);
        },
        (error) => reject(error)
      );
    });
  }
  static async getRosVersion() {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    let servicemaster = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/get_ros_version',
      serviceType: 'rosapi_msgs/srv/GetRosVersion',
    });

    let request = {};
    return new Promise((resolve, rejects) => {
      servicemaster.callService(
        request,
        function (result) {
          resolve(result.ros_version);
        },
        function (error) {
          console.error('Error getting ros version:', error);
          rejects(error);
        }
      );
    });
  }

  static async getPublishers(topic) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    let servicemaster = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/publishers',
      serviceType: 'rosapi_msgs/srv/Publishers',
    });

    let request ={ topic: topic };

    return new Promise((resolve, rejects) => {
      servicemaster.callService(
        request,
        function (result) {
          console.log(result);
          resolve(result.publishers);
        },
        function (error) {
          console.error('Error getting publishers:', error);
          rejects(error);
        }
      );
    });
  }

  static async PubRosMsg(params) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    const { topic, messageType, message } = params;

    const msgStructure = await rosModel.getMessageDetails(messageType);

    const typeMap = buildTypeMap(msgStructure);

    validateRosMsg(messageType, message, typeMap);

    const subscribers = await rosModel.getTopics();
    if (!subscribers.topics.includes(topic)) {
      throw new Error(`No subscribers found for topic '${topic}'`);
    }

    const pub = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: messageType,
    });
    const rosMsg = encodeRosSrv({ type: '', msg: message, msgType: messageType });
    if (!rosMsg) {
      throw new Error('ROS message is empty or invalid');
    }

    pub.on('warning', function (warning) {
      console.error('Advertencia:', warning);
    });

    pub.publish(rosMsg);
    return { topic: topic, msgType: messageType, msg: 'Message published successfully' };
  }

  static async subscribeOnce({ topic, messageType, timeout = 2000 }) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    // Verificar que el topic exista en la lista de topics activos
    const topics = await rosModel.getTopics();
    if (!topics.topics.includes(topic)) {
      return Promise.reject(new Error(`Topic '${topic}' does not exist`));
    }

    // Nota: /rosapi/publishers a menudo devuelve lista vacía incluso cuando hay publishers
    // Por eso verificamos solo la existencia del topic
    const publisher = await rosModel.getPublishers(topic);
    if (publisher.length === 0) {
      console.warn(
        `Warning: /rosapi/publishers returned empty list for topic '${topic}', but topic exists. Proceeding anyway...`
      );
    }

    const sub = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: messageType,
    });

    return new Promise((resolve, reject) => {
      const handler = (message) => {
        sub.unsubscribe();
        resolve(message); // Devuelve el mensaje recibido
      };

      sub.subscribe(handler);

      setTimeout(() => {
        sub.unsubscribe();
        reject(new Error('Timeout exceeded'));
      }, timeout);
    });
  }

  /*
   / Create Ros service server that UAV can consume for 
   / Check if UAV finish mission
   / Check if UAV finish Download
  */
  static GCSServicesMission() {
    const service_list = [
      {
        name: 'ServiceFinishMission',
        serviceName: '/GCS/FinishMission',
        serviceType: 'aerialcore_common/finishMission',
        callback: function (request, response) {
          console.log(`callback Sevice finish mission: ${JSON.stringify(request)}`);
          if (request.hasOwnProperty('uav_id')) {
              missionController.deviceFinishMission({ name: request.uav_id });
          }
          Object.assign(response, { success: true, msg: 'Set successfully' });
          return true;
        },
      },
      { name: 'ServiceDownload',
        serviceName: '/GCS/FinishDownload',
        serviceType: 'aerialcore_common/finishGetFiles',
        callback: function (request, response) {
          console.log(`callback Service finish download files: ${JSON.stringify(request)}`);
          if (request.hasOwnProperty('uav_id')) {
            missionController.deviceFinishSyncFiles({ name: request.uav_id });
          }
          Object.assign(response, { success: true, msg: 'Set successfully' });
          return true;
        },
      },
    ];

    for (let srv of service_list) {
      service_list[srv.name] = rosModel.serviceServer({
        serviceName: srv.serviceName,
        serviceType: srv.serviceType,
        callback: srv.callback,
      });  
    }
  }

  static serviceServer({ serviceName, serviceType, callback } ) {
    const service = new ROSLIB.Service({
      ros: ros,
      name: serviceName,
      serviceType: serviceType,
    });
    service.advertise(function (request, response) {
      callback(request, response);
    });
    return service;
  }

  static GCSunServicesMission() {
    for (let srv of service_list) {
      service_list[srv.name].unadvertise();
      delete service_list[srv.name];
    }
  }
  // actions Ros
  // https://github.com/sathak93/roslibjs/blob/ros2actionclient/examples/action.html
  static async getActionServer() {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    return new Promise((resolve, reject) => {
      ros.getActionServers(
        (actions) => resolve(actions),
        (error) => reject(error)
      );
    });
  }
  static async getActionGoalmsg(actionServer) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    let servicemaster = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/action_goal_details',
      serviceType: 'rosapi_msgs/srv/ActionGoalDetails',
    });

    let request = { type: actionServer };

    return new Promise((resolve, rejects) => {
      servicemaster.callService(
        request,
        function (result) {
          resolve(result.publishers);
        },
        function (error) {
          console.error('Error getting publishers:', error);
          rejects(error);
        }
      );
    });
  }

  static async sendActionGoal(args) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    const { action, actionType, message } = args;
    console.log(`Sending goal to action server '${action}' of type '${actionType}' with message:`, message);
    // action ros2 for roslibjs
    let newClient = new ROSLIB.Action({
      ros: ros,
      name: action,
      actionType: actionType,
    });

    let goal_id = newClient.sendGoal(message,
      (result) => {
          console.log(`✅ Resultado: ${JSON.stringify(result)}`);
          if (result.result && result.status === 4) {
            console.log('🎯 Navegación completada!');
          }else{
            console.log('❌ La navegación falló o fue cancelada.');
          }
        },
        (feedback) => {
          console.log(`📍 Feedback: ${JSON.stringify(feedback)}`);
        },
      (error)=>{
        console.error('action goal failed:', error);
      }
    );
    console.log('Goal sent with ID:', goal_id);

    const goalHandle = { id: "a" };

    // const nav2Client = new ROS2GoalActionClient(ros, action, actionType, true);
    // // Enviar goal
    // const goalHandle = nav2Client.sendGoal(
    //   { ...message },
    //   {
    //     onFeedback: (feedback) => {
    //       console.log(`📍 Feedback: ${JSON.stringify(feedback)}`);
    //     },
    //     onResult: (result) => {
    //       console.log(`✅ Resultado: ${JSON.stringify(result)}`);
    //       if (result.result && result.status === 4) {
    //         console.log('🎯 Navegación completada!');
    //       }else{
    //         console.log('❌ La navegación falló o fue cancelada.');
    //       }
    //     },
    //   }
    // );
    return { state: 'success', msg: 'Action goal sent successfully' , goalId: goalHandle.id};
  }

  static async cancelActionGoal(args) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    const { action, actionType, goalId } = args;

    const nav2Client = new ROS2GoalActionClient(ros, action, actionType, true);
    // Cancelar goal
    nav2Client.cancelGoal(goalId);
    return { state: 'success', msg: 'Action goal canceled successfully' };
  }

  // utilities ROS
  static async getActionServers() {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    return new Promise((resolve, reject) => {
      ros.getActionServers(
        (servers) => resolve(servers),
        (error) => reject(error)
      );
    });
  }

  static Getservicehost(nameService) {
    let servicehost = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/service_host',
      serviceType: 'rosapi/ServiceHost',
    });

    let request = { service: nameService };

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

    let request = {};

    return new Promise((resolve, rejects) => {
      servicemaster.callService(request, function (result) {
        console.log('masterip -- ' + result.length);
        resolve(result);
      });
    });
  }
}
