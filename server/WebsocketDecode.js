import * as fb from './dist/schema_main.cjs';
import { ServiceResponse } from './WebsocketDevices.js';

export function getNameFromTopic(topic) {
  // Absolute vs relative path
  if (topic.startsWith('/')) {
    return topic.split('/')[1];
  } else {
    return topic.split('/')[0];
  }
}

export function decoder(metadata, buf, deviceId, category, deviceName) {
  if (metadata.type() === 'sensor_msgs/NavSatFix') {
    //console.log('received navsatfix message');
    const msg = fb.fb.sensor_msgs.NavSatFix.getRootAsNavSatFix(buf);
    return {
      deviceId,
      latitude: msg.latitude(),
      longitude: msg.longitude(),
      altitude: msg.altitude(),
    };
  }
  if (metadata.type() === 'std_msgs/Float64' && metadata.topic().includes('orientation')) {
    const msg = fb.fb.std_msgs.Float64.getRootAsFloat64(buf);
    return { deviceId, course: msg.data() };
  }

  if (metadata.type() === 'sensor_msgs/Imu') {
    const msg = fb.fb.geometry_msgs.Vector3.getRootAsVector3(buf);
    return { deviceId, course: 90 + msg.z() * 57.295 };
  }
  if (metadata.type() === 'sensor_msgs/BatteryState' && category == 'px4') {
    return { deviceId, batteryLevel: (msg.percentage * 100).toFixed(0) };
  }
  if (metadata.type() === 'sensor_msgs/BatteryState') {
    const msg = fb.fb.sensor_msgs.BatteryState.getRootAsBatteryState(buf);
    return { deviceId, batteryLevel: msg.percentage().toFixed(0) };
  }
  if (metadata.type() === 'geometry_msgs/Vector3Stamped' && metadata.topic().includes('gimbal')) {
    const msg = fb.fb.geometry_msgs.Vector3Stamped.getRootAsVector3Stamped(buf);
    return {
      deviceId,
      gimbal: { x: msg.vector().x(), y: msg.vector().y(), z: msg.vector().z() },
    };
  }
  if (
    metadata.type() === 'geometry_msgs/Vector3Stamped' &&
    metadata.topic().includes('speed') &&
    !category.includes('dji_M300')
  ) {
    const msg = fb.fb.geometry_msgs.Vector3Stamped.getRootAsVector3Stamped(buf);
    return {
      deviceId,
      speed: Math.sqrt(Math.pow(msg.vector().x(), 2) + Math.pow(msg.vector().y(), 2)).toFixed(2),
    };
  }
  if (metadata.type() === 'std_msgs/UInt8' && metadata.topic().includes('flight_status')) {
    const msg = fb.fb.std_msgs.UInt8.getRootAsUInt8(buf);
    return { deviceId, protocol: 'dji', landed_state: msg.data() };
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
    return { deviceId, obstacle_info: obst_msg };
  }
  if (metadata.type() === 'geometry_msgs/TwistStamped' && metadata.topic().includes('speed')) {
    const msg = fb.fb.geometry_msgs.TwistStamped.getRootAsTwistStamped(buf);
    return {
      deviceId,
      speed: Math.sqrt(Math.pow(msg.twist().linear().x(), 2) + Math.pow(msg.twist().linear().y(), 2)).toFixed(2),
    };
  }
  if (metadata.type() === 'dji_osdk_ros/WaypointV2MissionStatePush') {
    const msg = fb.fb.dji_osdk_ros.WaypointV2MissionStatePush.getRootAsWaypointV2MissionStatePush(buf);
    return { deviceId, speed: msg.velocity() };
  }
  if (metadata.type() === 'std_msgs/UInt8' && metadata.topic().includes('alarm')) {
    const msg = fb.fb.std_msgs.UInt8.getRootAsUInt8(buf);
    return { deviceId, threat: msg.data() };
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
  return null;
}
