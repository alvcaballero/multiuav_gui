import ROSLIB from 'roslib';
import { DevicesModel } from "./devices";
var ros = '';
export { ros }
const rosState = { state: 'disconnect', msg: 'init msg' };
console.log('out of  device model');

const autoconectRos = setInterval(() => {
  if (rosState.state != 'connect') {
    console.log('try to connect ros');
    rosModel.rosConnect();
  }
}, 30000);

console.log("model ROS")
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
        this.setrosState({ state: 'connect', msg: 'Conectado a ROS' });
        DevicesModel.connectAllUAV();
      });
      ros.on('error', function (error) {
        console.log('ROS Error connecting to websocket server: ', error);
        this.setrosState({ state: 'error', msg: 'No se ha posido conectar a ROS' });
      });
      ros.on('close', function () {
        console.log('ROS Connection to websocket server closed.');
        this.setrosState({ state: 'disconnect', msg: 'Desconectado a ROS' });
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
