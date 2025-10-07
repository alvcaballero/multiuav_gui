import ROSLIB from 'roslib';
import { readDataFile, getDatetime } from '../common/utils.js';
import { devicesController } from '../controllers/devices.js';
import { missionController } from '../controllers/mission.js';
import { positionsController } from '../controllers/positions.js';
import { decodeRosMsg } from '../models/rosDecode.js';
import { encodeRosSrv } from '../models/rosEncode.js';
import { buildTypeMap, validateRosMsg } from '../models/rosValidateMSG.js';
import { categoryController } from '../controllers/category.js';
import logger, { logHelpers } from '../common/logger.js';

var ros = null;
const rosState = { state: 'disconnect', msg: 'init msg' };
const devices_msg = readDataFile('../config/devices/devices_msg.yaml');
const service_list = {};
const uav_list = {};

var autoconectRos = null;
var noTimerflag = true;

function connectRos() {
  //console.log('autoconectRos,' + noTimerflag + ' ros status is: ' + rosState.state);
  if (rosState.state != 'connect') {
    rosModel.rosConnect();
  } else {
    //console.log('clearInterval');
    clearInterval(autoconectRos);
    noTimerflag = true;
  }
}
function autoConectRos() {
  //console.log('notimerflag,' + noTimerflag);
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
        //const symbols = Object.getOwnPropertySymbols(error); // Obtener todos los símbolos del objeto
        //const kMessageSymbol = symbols.find((symbol) => symbol.toString() === 'Symbol(kMessage)'); // Buscar el símbolo específico
        //if (kMessageSymbol) {
        //  console.log('errora:' + error[kMessageSymbol]);
        //} else {
        //  console.log('errorb:' + error);
        //}
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
    //console.log('camera' + type);
    uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
      //console.log('camera suscribe');
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
        console.log('camera websocket');
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
    if (topicType !== messageType) {
      throw new Error(`Service type mismatch: expected '${topicType}', got '${messageType}'`);
    }

    const responseSrvStructure = await rosModel.getServiceRequestDetails(messageType);
    const srvStructure = responseSrvStructure.typedefs || [];

    if (!srvStructure || srvStructure.length === 0) {
      throw new Error(`No structure found for service type '${messageType}'`);
    }

    const typeMap = buildTypeMap(srvStructure)

    validateRosMsg(messageType, message, typeMap)

    let Message = new ROSLIB.Service({
      ros: ros,
      name: service,
      serviceType: messageType,
    });

    let MsgRequest = message ? new ROSLIB.ServiceRequest(message) : {};

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
      const response = await rosModel.callRosService({ service, messageType:msgType, message: myRequest });
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
        (serviceDetails) =>{
          if(!serviceDetails || (serviceDetails.length === 0)) {
            reject(new Error('Service type not found'));
          }
          resolve(serviceDetails)
        }
        ,
        (error) => reject(error)
      );
    });
  }
  static async getServiceResponseDetails(type) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');
    return new Promise((resolve, reject) => {
      ros.getServiceResponseDetails(
        type,
        (serviceDetails) =>{
          if(!serviceDetails || (serviceDetails.length === 0)) {
            reject(new Error('Service type not found'));
          }
          resolve(serviceDetails)
        }
        ,
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
        (messageDetails) =>{
          if(!messageDetails || (Array.isArray(messageDetails) && messageDetails.length === 0)) {
            reject(new Error('Message type not found'));
          }
          resolve(messageDetails)
        },
        (error) => reject(error)
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

    let request = new ROSLIB.ServiceRequest({ topic: topic });

    return new Promise((resolve, rejects) => {
      servicemaster.callService(request, function (result) {
        resolve(result.publishers);
      }, function (error) {
        console.error('Error getting publishers:', error);
        rejects(error);
      });
    });
  }

  static async PubRosMsg(params) {
    if (!ros || !ros.isConnected) throw new Error('ROS not connected');

    const { topic, messageType, message } = params;

    const msgStructure = await rosModel.getMessageDetails(messageType)
    
    const typeMap = buildTypeMap(msgStructure)

    validateRosMsg(messageType, message, typeMap)

    const subscribers = await rosModel.getTopics();
    if (!subscribers.topics.includes(topic)) {
      throw new Error(`No subscribers found for topic '${topic}'`);
    }

    const pub = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: messageType,
    });
    const msg = encodeRosSrv({ type: '', msg: message, msgType: messageType });
    const rosMsg = new ROSLIB.Message(msg);
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

    const publisher = await rosModel.getPublishers(topic);
    if (publisher.length === 0) {
      //console.warn('No publishers found for topic:', topic);
      return Promise.reject(new Error(`No publishers found for topic '${topic}'`));
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
   / Create Ros services that UAV can consume for 
   / Check if UAV finish mission
   / Check if UAV finish Download
  */
  static GCSServicesMission() {
    service_list['ServiceMission'] = new ROSLIB.Service({
      ros: ros,
      name: '/GCS/FinishMission',
      serviceType: 'aerialcore_common/finishMission',
    });

    service_list['ServiceMission'].advertise(function (request, response) {
      // Call state machine
      console.log('callback Sevice finish mission');
      console.log(request);
      if (request.hasOwnProperty('uav_id')) {
        if (Number.isNaN(Number.parseInt(request.uav_id))) {
          MissionController.deviceFinishMission({ name: request.uav_id });
        } else {
          MissionController.deviceFinishMission({ name: `uav_${request.uav_id}` });
        }
      }
      response['success'] = true;
      response['msg'] = 'Set successfully';
      return true;
    });

    service_list['ServiceDownload'] = new ROSLIB.Service({
      ros: ros,
      name: '/GCS/FinishDownload',
      serviceType: 'aerialcore_common/finishGetFiles',
    });

    service_list['ServiceDownload'].advertise(function (request, response) {
      console.log('callback Service finish download files');
      console.log(request);
      if (request.hasOwnProperty('uav_id')) {
        MissionController.deviceFinishSyncFiles({ name: request.uav_id });
      }
      response['success'] = true;
      response['msg'] = 'Set successfully';
      return true;
    });
  }
  static GCSunServicesMission() {
    //console.log('Unadvertise services');
    if (service_list['ServiceMission']) {
      service_list['ServiceMission'].unadvertise();
      service_list['ServiceDownload'].unadvertise();
      delete service_list['ServiceMission'];
      delete service_list['ServiceDownload'];
    }
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
