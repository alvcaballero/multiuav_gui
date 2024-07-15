import * as fb from './dist/schema_main.cjs';
import { positionsController } from './controllers/positions.js';
import { DevicesController } from './controllers/devices.js';
import { ServiceResponse } from './WebsocketDevices.js';

function getNameFromTopic(topic) {
  // Absolute vs relative path
  if (topic.startsWith('/')) {
    return topic.split('/')[1];
  } else {
    return topic.split('/')[0];
  }
}

export async function decoder(metadata, buf, name) {
  let deviceName = name;
  //console.log('name of device:' + name);
  if (name == null) {
    deviceName = getNameFromTopic(metadata.topic());
    console.log('name space of device:' + name);
  }

  let device = await DevicesController.getByName(deviceName);
  if (!device) {
    console.log('device not found');
    return;
  }
  let deviceId = device.id;
  if (metadata.type() === 'sensor_msgs/NavSatFix') {
    //console.log('received navsatfix message');
    const msg = fb.fb.sensor_msgs.NavSatFix.getRootAsNavSatFix(buf);
    positionsController.updatePosition({
      deviceId,
      latitude: msg.latitude(),
      longitude: msg.longitude(),
      altitude: msg.altitude(),
    });
    return null;
  }
  if (metadata.type() === 'sensor_msgs/Imu') {
    const msg = fb.fb.geometry_msgs.Vector3.getRootAsVector3(buf);
    positionsController.updatePosition({ deviceId, course: 90 + msg.z() * 57.295 });
    return null;
  }
  if (metadata.type() === 'sensor_msgs/BatteryState') {
    const msg = fb.fb.sensor_msgs.BatteryState.getRootAsBatteryState(buf);
    positionsController.updatePosition({
      deviceId,
      batteryLevel: msg.percentage().toFixed(0),
    }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
    return null;
  }
  if (metadata.type() === 'geometry_msgs/Vector3Stamped' && metadata.topic().includes('gimbal')) {
    const msg = fb.fb.geometry_msgs.Vector3Stamped.getRootAsVector3Stamped(buf);
    positionsController.updatePosition({
      deviceId,
      gimbal: { x: msg.vector().x(), y: msg.vector().y(), z: msg.vector().z() },
    }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
    return null;
  }
  if (
    metadata.type() === 'geometry_msgs/Vector3Stamped' &&
    metadata.topic().includes('speed') &&
    !device.category.includes('dji_M300')
  ) {
    const msg = fb.fb.geometry_msgs.Vector3Stamped.getRootAsVector3Stamped(buf);
    positionsController.updatePosition({
      deviceId,
      speed: Math.sqrt(Math.pow(msg.vector().x(), 2) + Math.pow(msg.vector().y(), 2)).toFixed(2),
    }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
    return null;
  }
  if (metadata.type() === 'std_msgs/UInt8' && metadata.topic().includes('flight_status')) {
    const msg = fb.fb.std_msgs.UInt8.getRootAsUInt8(buf);
    positionsController.updatePosition({
      deviceId,
      protocol: 'dji',
      landed_state: msg.data(),
    });
    return null;
  }
  if (metadata.type() === 'dji_osdk_ros/ObstacleInfo') {
    const msg = fb.fb.dji_osdk_ros.ObstacleInfo.getRootAsObstacleInfo(buf);
    let obst_msg = {
      down: msg.down(),
      front: msg.front(),
      right: msg.right(),
      back: msg.back(),
      left: msg.left(),
      up: msg.up(),
    };
    positionsController.updatePosition({ deviceId, obstacle_info: obst_msg }); // showData[3].innerHTML = message.percentage + "%";
    return null;
  }
  if (metadata.type() === 'dji_osdk_ros/WaypointV2MissionStatePush') {
    const msg = fb.fb.dji_osdk_ros.WaypointV2MissionStatePush.getRootAsWaypointV2MissionStatePush(buf);
    positionsController.updatePosition({
      deviceId,
      speed: msg.velocity() * 0.01,
    });
    return null;
  }
  if (metadata.type() === 'std_srvs/SetBool' && metadata.topic().includes('commandMission')) {
    const msg = fb.fb.std_srvs.SetBool.getRootAsSetBool(buf);
    let success = msg.response().success();
    console.log('response setbool: %s %s', success, msg.response().message());
    let response = {};
    if (success) {
      response = { state: 'success', msg: 'command mission' + ' to' + deviceName + ' ok' };
    } else {
      response = { state: 'error', msg: 'command mission' + ' to' + deviceName + ' fail' };
    }
    ServiceResponse({
      uav_id: deviceId,
      name: deviceName,
      type: 'commandMission',
      response: response,
    });
    return null;
  }
  if (metadata.type() === 'aerialcore_common/ConfigMission' && metadata.topic().includes('configureMission')) {
    const msg = fb.fb.aerialcore_common.ConfigMission.getRootAsConfigMission(buf);
    let success = msg.response().success();
    console.log('response Config mission: %s ', success);
    let response = {};
    if (success) {
      response = { state: 'success', msg: 'Config mission' + ' to' + deviceName + ' ok' };
    } else {
      response = { state: 'error', msg: 'Config mission' + ' to' + deviceName + ' fail' };
    }
    ServiceResponse({
      uav_id: deviceId,
      name: deviceName,
      type: 'configureMission',
      response: response,
    });
    return null;
  }

  console.log('can not decode msg: %s type: %s', metadata.topic(), metadata.type());
}
