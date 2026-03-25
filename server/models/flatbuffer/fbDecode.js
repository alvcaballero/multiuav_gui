import * as fb from 'fbmsglib';

export function getNameFromTopic(topic) {
  if (topic.startsWith('/')) {
    return topic.split('/')[1];
  }
  return topic.split('/')[0];
}

export function decodeFbMsg(metadata, buf, deviceId, category, deviceName) {
  if (metadata.type() === 'sensor_msgs/NavSatFix') {
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
    // FIXED: was getRootAsVector3 with wrong field z; Imu carries orientation as quaternion
    // const msg = fb.fb.geometry_msgs.Vector3.getRootAsVector3(buf);
    // return { deviceId, course: 90 + msg.z() * 57.295 };
    const msg = fb.fb.sensor_msgs.Imu.getRootAsImu(buf);
    const q = msg.orientation();
    // yaw from quaternion → degrees [0, 360), NED convention (0=North, 90=East)
    const yawRad = Math.atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    return { deviceId, course: (yawRad * 57.2958 + 360) % 360 };
  }
  if (metadata.type() === 'sensor_msgs/BatteryState' && category === 'px4') {
    const msg = fb.fb.sensor_msgs.BatteryState.getRootAsBatteryState(buf);
    return { deviceId, batteryLevel: (msg.percentage() * 100).toFixed(0) };
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
    return {
      deviceId,
      obstacle_info: {
        down: msg.down(),
        front: msg.front(),
        right: msg.right(),
        back: msg.back(),
        left: msg.left(),
        up: msg.up(),
      },
    };
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

  return { deviceId: null, type: metadata.type(), topic: metadata.topic(), deviceName };
}

export function isServiceResponse(metadata) {
  return (
    (metadata.type() === 'std_srvs/SetBool' && metadata.topic().includes('commandMission')) ||
    (metadata.type() === 'aerialcore_common/ConfigMission' && metadata.topic().includes('configureMission'))
  );
}

export function decodeServiceResponse(metadata, buf, deviceId, deviceName) {
  if (metadata.type() === 'std_srvs/SetBool' && metadata.topic().includes('commandMission')) {
    const msg = fb.fb.std_srvs.SetBool.getRootAsSetBool(buf);
    const success = msg.response().success();
    return {
      uav_id: deviceId,
      name: deviceName,
      type: 'commandMission',
      response: {
        state: success ? 'success' : 'error',
        msg: `command mission to ${deviceName} ${success ? 'ok' : 'fail'}`,
      },
    };
  }
  if (metadata.type() === 'aerialcore_common/ConfigMission' && metadata.topic().includes('configureMission')) {
    const msg = fb.fb.aerialcore_common.ConfigMission.getRootAsConfigMission(buf);
    const success = msg.response().success();
    return {
      uav_id: deviceId,
      name: deviceName,
      type: 'configureMission',
      response: {
        state: success ? 'success' : 'error',
        msg: `Config mission to ${deviceName} ${success ? 'ok' : 'fail'}`,
      },
    };
  }
  return null;
}
