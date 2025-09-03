import { getDatetime, round } from '../common/utils.js';
export function decodeRosMsg({ msg, deviceId, uav_type, type, msgType }) {
  // console.log(`Decoding ROS message: ${type} ${msgType} for device ${deviceId}`);
  if (type == 'position' && msgType == 'sensor_msgs/NavSatFix') {
    return {
      id: msg.header.seq,
      deviceId,
      latitude: msg.latitude,
      longitude: msg.longitude,
      altitude: round(msg.altitude, 1),
      deviceTime: getDatetime(), // "2023-03-09T22:12:44.000+00:00",
    };
  }
  if (type == 'position_gps' && msgType == 'px4_msgs/msg/SensorGps') {
    return {
      id: 0,
      deviceId,
      latitude: msg.latitude_deg,
      longitude: msg.longitude_deg,
      altitude: round(msg.altitude_msl_m, 1),
      deviceTime: getDatetime(), // "2023-03-09T22:12:44.000+00:00",
    };
  }
  if (type == 'position_local' && msgType == 'px4_msgs/msg/VehicleLocalPosition') {
    return {
      id: 0,
      deviceId,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      yaw: msg.heading,
      deviceTime: getDatetime(), // "2023-03-09T22:12:44.000+00:00",
    };
  }
  //if (type == 'sensor_height' && msgType == 'mavros_msgs/Altitude') {
  //  return null;
  //}

  //if (type == 'sensor_height' && msgType == 'std_msgs/Float32') {
  //  return null; // return {deviceId:uav_id,altitude:msg.data};
  //}
  //if (type == 'vo_position' && msgType == 'dji_osdk_ros/VOPosition') {
  //  return null; // return {id:msg.header.seq,deviceId:uav_id,latitude:msg.x,longitude:msg.y,altitude:msg.z,course:0,deviceTime:"2023-03-09T22:12:44.000+00:00"};
  //}
  if (type == 'IMU' && msgType == 'sensor_msgs/Imu' && uav_type == 'sjtu') {
    // dji sdk
    //https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
    // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    //  q0, q1, q2, q3 corresponds to w,x,y,z
    let q = msg.orientation;
    let yaw = Math.atan2(2.0 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * -1;

    return {
      deviceId,
      course: Math.round(-90 + yaw * 57.295),
      orientation: { yaw: yaw, pitch: msg.orientation.x, roll: msg.orientation.y },
    };
  }
  if (type == 'IMU' && msgType == 'sensor_msgs/Imu') {
    // dji sdk
    //https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
    // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    //  q0, q1, q2, q3 corresponds to w,x,y,z
    let q = msg.orientation;
    let yaw = Math.atan2(2.0 * (q.z * q.w + q.x * q.y), -1 + +2 * (q.w * q.w + q.x * q.x)) * -1;

    return { deviceId, course: Math.round(90 + yaw * 57.295) };
  }
  if (type == 'hdg' && (msgType == 'std_msgs/Float64' || msgType == 'std_msgs/Float32')) {
    return { deviceId, course: msg.data };
  }
  if (type == 'mission_state' && msgType == 'dji_osdk_ros/WaypointV2MissionStatePush') {
    return { deviceId, speed: round(msg.velocity * 0.01, 1) };
  }

  if (type == 'speed' && msgType == 'geometry_msgs/Vector3Stamped' && !uav_type.includes('dji_M300')) {
    return {
      deviceId,
      speed: round(Math.sqrt(Math.pow(msg.vector.x, 2) + Math.pow(msg.vector.y, 2) + Math.pow(msg.vector.z, 2)), 1),
    };
  }
  if (type == 'speed' && msgType == 'geometry_msgs/TwistStamped') {
    console.log(`Speed message: ${JSON.stringify(msg)}`);
    return {
      deviceId,
      speed: round(
        Math.sqrt(Math.pow(msg.twist.linear.x, 2) + Math.pow(msg.twist.linear.y, 2) + +Math.pow(msg.twist.linear.z, 2)),
        1
      ),
    };
  }

  if (type == 'battery' && uav_type == 'px4') {
    return {
      deviceId,
      batteryLevel: Math.round(msg.percentage * 100),
    };
  }
  if (type == 'battery') {
    return { deviceId, batteryLevel: msg.percentage };
  }
  if (type == 'gimbal' && msgType == 'geometry_msgs/Vector3Stamped') {
    return { deviceId, gimbal: msg.vector };
  }
  if (type == 'obstacle_info' && msgType == 'dji_osdk_ros/ObstacleInfo') {
    return { deviceId, obstacle_info: msg };
  }
  if (type == 'obstacle_info' && msgType == 'sensor_msg/Range') {
    return { deviceId, obstacle_info: { down: msg.range } };
  }
  if (type == 'camera' && msgType == 'sensor_msgs/CompressedImage') {
    return { deviceId, camera: msg.data }; // {deviceId:uav_id,camera:"data:image/jpg;base64," + msg.data};
  }
  if (type == 'flight_status' && msgType == 'std_msgs/UInt8') {
    return {
      deviceId,
      protocol: 'dji',
      landed_state: msg.data,
    };
  }
  if (type == 'sensors_humidity') {
    return { deviceId, sensors_humidity: msg.data };
  }
  if (type == 'threat') {
    return { deviceId, threat: msg.data };
  }
  if (type == 'state_machine' && msgType == 'std_msgs/String') {
    return {
      deviceId,
      protocol: 'catec',
      mission_state: '0',
      wp_reached: '0',
      uav_state: 'ok',
      landed_state: msg.data,
    };
  }
  if (type == 'state_machine' && msgType == 'muav_state_machine/UAVState') {
    return {
      deviceId,
      protocol: msg.airframe_type,
      mission_state: msg.mission_state,
      wp_reached: msg.wp_reached,
      uav_state: msg.uav_state,
      landed_state: msg.landed_state,
    };
  }
  // console.log(`Unknown message type: ${type} ${msgType} for device ${deviceId}`);
  return null;
}


