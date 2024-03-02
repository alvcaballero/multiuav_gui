import { readJSON, readYAML, getDatetime } from '../common/utils.js';
import { positionsModel } from '../models/positions.js';
import { eventsModel } from '../models/events.js';
import ROSLIB from 'roslib';
import { ros, rosModel } from '../models/ros.js';
const devices_msg = readYAML('../config/devices/devices_msg.yaml');
const devices_init = readYAML('../config/devices/devices_init.yaml');
const devices = {};
const devicesAccess = {};
const uav_list = [];

const CheckDeviceOnline = setInterval(() => {
  let currentTime = new Date();
  let checkdevices = Object.keys(devices);
  checkdevices.forEach((element) => {
    //console.log(data.state.devices[element])
    //console.log(currentTime)
    if (currentTime - devices[element]['lastUpdate'] < 5000) {
      devices[element]['status'] = 'online';
    } else {
      devices[element]['status'] = 'offline';
    }
  });
}, 5000);

export class DevicesModel {
  constructor() {
    // conect with ros and other things
    console.log('constructor device model');
  }

  static async getAll(query) {
    if (query) {
      console.log(query);
      if (Array.isArray(query)) {
        const asArray = Object.entries(devices);
        const filtered = asArray.filter(([key, value]) => query.some((element) => key == element));
        return Object.fromEntries(filtered);
      }
      if (!isNaN(query)) {
        return devices[query] ? { id: devices[query] } : {};
      }
    }

    return devices;
  }
  static getAccess(id) {
    return Object.values(devicesAccess).find((device) => device.id === id);
  }

  static async create(device) {
    console.log(device);
    let serverState = rosModel.serverStatus();

    let uav_ns = device.name;
    let uav_type = device.category;
    let cur_uav_idx = String(Object.values(devices).length);

    console.log('create UAV');
    console.log('name' + uav_ns + '  type' + uav_type);

    let repeat_device = false;
    if (Object.values(devices).length > 0) {
      Object.values(devices).forEach((element) => {
        if (element) {
          repeat_device = element.name == uav_ns ? true : false;
        }
      });
    }

    if (repeat_device == true) {
      console.log('Dispositivo ya se encuentra registrado ' + uav_ns);
      return {
        state: 'error',
        msg: `Dispositivo ya se encuentra registrado ${uav_ns}`,
      };
    }

    this.updatedevice({
      id: cur_uav_idx,
      name: device.name,
      category: device.category,
      ip: device.ip,
      camera: device.camera,
      status: 'online',
      lastUpdate: null,
    });

    if (device.hasOwnProperty('user') && device.hasOwnProperty('pwd')) {
      this.updateDeviceAccess({
        id: cur_uav_idx,
        name: device.name,
        user: device.user,
        pwd: device.pwd,
        ip: device.ip,
      });
    }
    await this.addCameraWebRTC(device);

    if (serverState.state === 'connect') {
      console.log('suscribe devices ');
      await this.subscribeDevice({
        id: cur_uav_idx,
        name: uav_ns,
        type: uav_type,
        camera: device.camera,
        watch_bound: true,
        bag: false,
      });

      console.log(devices);
      console.log('success', device.name + ' added. Type: ' + device.category);
      return { state: 'success', msg: 'conectado Correctamente' };
    } else {
      console.log('\nRos no está conectado.\n\n Por favor conéctelo primero.');
      return { state: 'error', msg: 'Ros no está conectado' };
    }
  }

  static async addCameraWebRTC(device) {
    for (let i = 0; i < device.camera.length; i = i + 1) {
      if (device.camera[i]['type'] == 'WebRTC') {
        await fetch(
          `http://localhost:9997/v3/config/paths/add/${device.name}_${device.camera[i].source}`,
          {
            method: 'POST',
            body: JSON.stringify({
              source: `rtsp://${device.ip}:8554/${device.camera[i].source}`,
            }),
            headers: {
              'Content-Type': 'application/json',
            },
          }
        );
      }
    }
  }
  static async subscribeDevice(uavAdded) {
    uav_list.push(uavAdded);
    console.log(uavAdded);
    let uav_type = uavAdded.type;
    let cur_uav_idx = uavAdded.id;
    let uav_ns = uavAdded.name;
    let uav_camera = uavAdded.camera;
    // Subscribing
    // create subcribin mesage
    Object.keys(devices_msg[uav_type]['topics']).forEach((element) => {
      console.log(element);
      uav_list[cur_uav_idx]['listener_' + element] = new ROSLIB.Topic({
        ros: ros,
        name: uav_ns + devices_msg[uav_type]['topics'][element]['name'], //'/dji_osdk_ros/rtk_position',
        messageType: devices_msg[uav_type]['topics'][element]['messageType'],
      });
    });
    // DJI
    if (
      uav_type == 'dji_M210_noetic' ||
      uav_type == 'dji_M210_melodic' ||
      uav_type == 'dji_M300' ||
      uav_type == 'dji_M210_noetic_rtk' ||
      uav_type == 'dji_M210_melodic_rtk'
    ) {
      uav_list[cur_uav_idx].listener_position.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({
          id: msg.header.seq,
          deviceId: id_uav,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: getDatetime(), // "2023-03-09T22:12:44.000+00:00",
        });
      });

      uav_list[cur_uav_idx].listener_sensor_height.subscribe(function (msg) {
        //Altitud de ultrasonico
        //let id_uav = cur_uav_idx;
        //positionsModel.updatePosition({deviceId:id_uav,altitude:msg.data});
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
        let yaw = Math.atan2(2.0 * (q.x * q.y + q.w * q.z), -1 + 2 * (q.w * q.w + q.x * q.x)) * -1;
        positionsModel.updatePosition({ deviceId: id_uav, course: 90 + yaw * 57.295 });
      });

      if (uav_type == 'dji_M300') {
        uav_list[cur_uav_idx].listener_mission_state.subscribe(function (msg) {
          let id_uav = cur_uav_idx; // var showData = document.getElementById(uav_ns).cells;
          positionsModel.updatePosition({
            deviceId: id_uav,
            speed: msg.velocity * 0.01,
          });
        });
        uav_list[cur_uav_idx].listener_flight_status.subscribe(function (msg) {
          let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
          positionsModel.updatePosition({
            deviceId: id_uav,
            protocol: 'dji',
            landed_state: msg.data,
          }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
        });
      } else {
        uav_list[cur_uav_idx].listener_speed.subscribe(function (msg) {
          let id_uav = cur_uav_idx; // var showData = document.getElementById(uav_ns).cells;
          positionsModel.updatePosition({
            deviceId: id_uav,
            speed: Math.sqrt(Math.pow(msg.vector.x, 2) + Math.pow(msg.vector.y, 2)).toFixed(2),
          }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.vector.x,2) + Math.pow(message.vector.y,2)).toFixed(2);
        });
      }

      uav_list[cur_uav_idx].listener_battery.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({ deviceId: id_uav, batteryLevel: msg.percentage }); // showData[3].innerHTML = message.percentage + "%";
      });

      uav_list[cur_uav_idx].listener_gimbal.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({ deviceId: id_uav, gimbal: msg.vector }); // showData[3].innerHTML = message.percentage + "%";
      });

      uav_list[cur_uav_idx].listener_obstacle_info.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({ deviceId: id_uav, obstacle_info: msg }); // showData[3].innerHTML = message.percentage + "%";
      });

      for (let i = 0; i < uav_camera.length; i = i + 1) {
        if (uav_camera[i]['type'] == 'Websocket') {
          uav_list[cur_uav_idx].listener_camera.subscribe(function (msg) {
            let id_uav = cur_uav_idx; //console.log("dato camara"+ id_uav + "--"+ msg.data)
            positionsModel.updateCamera({ deviceId: id_uav, camera: msg.data }); //positionsModel.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data});//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
          });
        }
      }
    } else if (uav_type == 'px4') {
      uav_list[cur_uav_idx].listener_position.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({
          id: msg.header.seq,
          deviceId: id_uav,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: getDatetime(), //"2023-03-09T22:12:44.000+00:00",
        });
      });

      uav_list[cur_uav_idx].listener_hdg.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({ deviceId: id_uav, course: msg.data }); //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
      });

      uav_list[cur_uav_idx].listener_sensor_height.subscribe(function (message) {
        //var showData = document.getElementById(uav_ns).cells;
        //showData[1].innerHTML = (message.relative).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_speed.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
          deviceId: id_uav,
          speed: Math.sqrt(
            Math.pow(msg.twist.linear.x, 2) + Math.pow(msg.twist.linear.y, 2)
          ).toFixed(2),
        }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_battery.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
          deviceId: id_uav,
          batteryLevel: (msg.percentage * 100).toFixed(0),
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });

      uav_list[cur_uav_idx].listener_state_machine.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
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
        positionsModel.updateCamera({ deviceId: id_uav, camera: msg.data }); //positionsModel.updateCamera({deviceId:id_uav,camera:"data:image/jpg;base64," + msg.data});//document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;//document.getElementById('my_image').src = "data:image/bgr8;base64," + message.data;
      });

      uav_list[cur_uav_idx].listener_threat.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({ deviceId: id_uav, threat: msg.data });
      });

      //let ipdevice = await Getservicehost(uav_ns+'/mavros/mission/clear');
      //console.log("ip de uav"+ ipdevice);
      //positionsModel.updatedeviceIP({id: cur_uav_idx,ip:ipdevice});
    } else if (uav_type == 'fuvex') {
      uav_list[cur_uav_idx].listener_position.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({
          id: msg.header.seq,
          deviceId: id_uav,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: getDatetime(), //"2023-03-09T22:12:44.000+00:00",
        });
      });
      uav_list[cur_uav_idx].listener_hdg.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({ deviceId: id_uav, course: msg.data }); //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
      });
      uav_list[cur_uav_idx].listener_speed.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
          deviceId: id_uav,
          speed: Math.sqrt(
            Math.pow(msg.twist.linear.x, 2) + Math.pow(msg.twist.linear.y, 2)
          ).toFixed(2),
        }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_battery.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
          deviceId: id_uav,
          batteryLevel: (msg.percentage * 100).toFixed(0),
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });
    } else if (uav_type == 'catec') {
      uav_list[cur_uav_idx].listener_position.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({
          id: msg.header.seq,
          deviceId: id_uav,
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude,
          deviceTime: getDatetime(), //"2023-03-09T22:12:44.000+00:00",
        });
      });

      uav_list[cur_uav_idx].listener_hdg.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({ deviceId: id_uav, course: msg.data }); //uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
      });

      uav_list[cur_uav_idx].listener_speed.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
          deviceId: id_uav,
          speed: Math.sqrt(
            Math.pow(msg.twist.linear.x, 2) + Math.pow(msg.twist.linear.y, 2)
          ).toFixed(2),
        }); // showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
      });

      uav_list[cur_uav_idx].listener_battery.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
          deviceId: id_uav,
          batteryLevel: (msg.percentage * 100).toFixed(0),
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });

      uav_list[cur_uav_idx].listener_state_machine.subscribe(function (msg) {
        let id_uav = cur_uav_idx; //var showData = document.getElementById(uav_ns).cells;
        positionsModel.updatePosition({
          deviceId: id_uav,
          protocol: 'catec',
          mission_state: '0',
          wp_reached: '0',
          uav_state: 'ok',
          landed_state: msg.data,
        }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
      });

      uav_list[cur_uav_idx].listener_threat.subscribe(function (msg) {
        let id_uav = cur_uav_idx;
        positionsModel.updatePosition({ deviceId: id_uav, threat: msg.data });
        eventsModel.addEvent({
          type: 'warning',
          eventTime: getDatetime(),
          deviceId: id_uav,
          attributes: { message: self.get_device_ns(id_uav) + ':treat event' },
        });
      });
    }
  }

  static async getById({ id }) {
    return Object.values(devices).find((device) => device.id === id);
  }
  static getByName(name) {
    return Object.values(devices).find((device) => device.name === name);
  }

  static async delete({ id }) {
    console.log(id);
    await this.removeDeviceCameraWebRTC(id);
    await this.removedevice({ id: id });
    if (uav_list.length != 0) {
      await this.unsubscribe(id);

      return { state: 'success', msg: 'Se ha eliminado el ' + cur_uav_idx }; //notification('success',"Commanding mission to: " + cur_roster);
    } else {
      return { state: 'success', msg: 'no quedan UAV de la lista' };
    }
  }
  static async removeDeviceCameraWebRTC(id) {
    let device_del = devices[id];
    if (device_del.hasOwnProperty('camera')) {
      for (let i = 0; i < device_del.camera.length; i = i + 1) {
        if (device_del.camera[i]['type'] == 'WebRTC') {
          await fetch(
            `http://localhost:9997/v3/config/paths/remove/${device_del.name}_${device_del.camera[i].source}`,
            {
              method: 'POST',
            }
          );
        }
      }
    }
  }

  static async unsubscribe(id) {
    if (id < 0) {
      for (var i = 0; i < uav_list.length; i++) {
        uav_list[i].listener.unsubscribe();
      }
      uav_list = [];
    } else {
      const cur_uav_idx = uav_list.findIndex((element) => element.id == id);

      let Key_listener = Object.keys(uav_list[cur_uav_idx]).filter((element) =>
        element.includes('listener')
      );

      console.log(Key_listener);
      if (uav_list.length != 0) {
        Key_listener.forEach((element) => {
          console.log(element);
          uav_list[cur_uav_idx][element].unsubscribe();
        });
        uav_list[cur_uav_idx] = null;
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
  }

  static async update({ id, input }) {
    const movieIndex = movies.findIndex((movie) => movie.id === id);
    if (movieIndex === -1) return false;

    movies[movieIndex] = {
      ...movies[movieIndex],
      ...input,
    };

    return movies[movieIndex];
  }

  static updatedevice(payload) {
    devices[payload.id] = payload;
  }
  static updateDeviceAccess(payload) {
    devicesAccess[payload.id] = payload;
  }
  static updatedeviceIP(payload) {
    devices[payload.id]['ip'] = payload.ip;
  }
  static updatedevicetime(id) {
    let currentTime = new Date();
    devices[id]['lastUpdate'] = currentTime;
  }
  static get_device_ns(uav_id) {
    return devices[uav_id].name;
  }
  static get_device_category(uav_id) {
    return devices[uav_id].category;
  }

  static removedevice({ id }) {
    delete devices[id];
    console.log(devices);
  }

  static async connectAllUAV() {
    for (let device of Object.values(devices)) {
      await this.subscribeDevice({
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
  static async addAllUAV() {
    for (let device of devices_init.init) {
      let uno = await this.create(device);
    }
    console.log('finish to add all devices  ------------');
  }
}
DevicesModel.addAllUAV();
