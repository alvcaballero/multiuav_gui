import ROSLIB from 'roslib';
import { DevicesModel } from '../models/devices.js';
var ros = '';
export { ros };
const rosState = { state: 'disconnect', msg: 'init msg' };
console.log('out of  device model');

const autoconectRos = setInterval(() => {
  if (rosState.state != 'connect') {
    console.log('try to connect ros');
    rosModel.rosConnect();
  }
}, 30000);

console.log('model ROS');
export class rosModel {
  static setrosState({ state, msg }) {
    rosState['state'] = state;
    rosState['msg'] = msg;
  }
  static serverStatus() {
    return rosState;
  }

  static async rosConnect() {
    console.log('try to connect to ros');
    if (rosState.state != 'connect') {
      ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
      ros.on('connection', function () {
        console.log('ROS Connected to websocket server.');
        rosModel.setrosState({ state: 'connect', msg: 'Conectado a ROS' });
        DevicesModel.connectAllUAV();
      });
      ros.on('error', function (error) {
        console.log('ROS Error connecting to websocket server: ', error);
        rosModel.setrosState({ state: 'error', msg: 'No se ha posido conectar a ROS' });
      });
      ros.on('close', function () {
        console.log('ROS Connection to websocket server closed.');
        rosModel.setrosState({ state: 'disconnect', msg: 'Desconectado a ROS' });
      });
    } else {
      DevicesModel.unsubscribe(-1);
      ros.close();
    }
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
  static getTopics2() {

  }
  static ServiceMission() {
    var ServiceMission = new ROSLIB.Service({
      ros: ros,
      name: '/GCS/Mission',
      serviceType: 'std_srvs/SetBool',
    });

    
    var request = new ROSLIB.ServiceRequest();
    return new Promise((resolve, rejects) => {
      topicsClient.callService(request, function (result) {
        resolve(result.topics);
      });
    });

  }
  static getServices(){

      const servicesPromise = new Promise((resolve, reject) => {
        ros.ROS.getServices((services) => {
          const serviceList = services.map((serviceName) => {
            return {
              serviceName,
            }
          });
          resolve({
            services: serviceList
          });
          reject({
            services: []
          });
        }, (message) => {
          console.error("Failed to get services", message)
          ros.services = []
        });
      });
      servicesPromise.then((services) => setROS(ros => ({ ...ros, services: services })));
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
