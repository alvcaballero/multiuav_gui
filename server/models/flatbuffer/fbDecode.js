import * as fb from 'fbmsglib';
import { ServiceKey, SubscriberKey } from '../../config/deviceMsgCatalog.js';
import { djiFlightStatus, djiFlightMode } from '../vehicleStates.js';
import { decodeMsg, hasDecoder } from '../msgDecode.js';
import { fbMsgAsRos } from './fbUnpack.js';

export function getNameFromTopic(topic) {
  if (topic.startsWith('/')) {
    return topic.split('/')[1];
  }
  return topic.split('/')[0];
}

// The robofleet topic without its device prefix ('/uav1/gps_position' → 'gps_position'): what the
// raw inspector uses as the `slot` of a FlatBuffer device, whose topics are robofleet's own names
// and not the keys of devices_msg.
export function slotFromTopic(topic) {
  return topic.replace(/^\//, '').split('/').slice(1).join('/');
}

// Slot of a robofleet topic according to the device's category: the key of the `subscribers` entry
// whose `name` is that topic ('gps_position' ← position: {name: '/gps_position'}), the same
// '/<device>' + name rule ROS uses. A topic the category does not declare keeps its own name, so
// a device whose bridge has not been renamed yet is still inspectable.
export function slotOfTopic(subscribers, topic) {
  return findSlot(subscribers, topic) ?? slotFromTopic(topic);
}

// The slot the category declares for that topic, or null. No fallback: it decides whether a
// message goes through the category's decoder at all.
export function findSlot(subscribers, topic) {
  const suffix = '/' + slotFromTopic(topic);
  return Object.entries(subscribers ?? {}).find(([, entry]) => entry?.name === suffix)?.[0] ?? null;
}

// The categories name ROS2 types 'pkg/msg/Name'; robofleet carries 'pkg/Name'.
const sameMsgType = (a, b) => String(a).replace('/msg/', '/') === String(b).replace('/msg/', '/');

/**
 * Decodes a robofleet message the way every other transport does: unpack it into the ROS message
 * it carries and hand it to the `msgDecode` registry, picked by the slot the category declares for
 * its topic. So FlatBuffer, rosbridge and MQTT share one decoder per (slot, messageType).
 *
 * @returns {object|null|undefined} the TelemetryPatch; `null` when the decoder says there is
 *   nothing to apply (an explicit `ignore`); `undefined` when this message does not go through
 *   the category (undeclared topic, other type, no decoder, camera) and the caller should fall back
 *   to `decodeFbMsg`.
 */
export function decodeFbViaCategory({ type, topic, buf, deviceId, subscribers }) {
  const slot = findSlot(subscribers, topic);
  const entry = slot ? subscribers[slot] : null;
  if (!entry || slot === SubscriberKey.CAMERA) return undefined;
  if (!sameMsgType(entry.messageType, type) || !hasDecoder(slot, entry.messageType)) return undefined;

  const msg = fbMsgAsRos(type, buf);
  if (!msg) return undefined;
  return decodeMsg({ msg, deviceId, type: slot, msgType: entry.messageType, params: entry });
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
    return { deviceId, ...djiFlightStatus(msg.data()), flightPhaseRaw: msg.data() };
  }
  if (metadata.type() === 'std_msgs/UInt8' && metadata.topic().includes('display_mode')) {
    const msg = fb.fb.std_msgs.UInt8.getRootAsUInt8(buf);
    return { deviceId, flightMode: djiFlightMode(msg.data()), flightModeRaw: msg.data() };
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

// The topics are robofleet's own names (/commandMission, /configureMission); `type` maps them
// back to the devices_msg key the request was sent under.
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
      type: ServiceKey.COMMAND_MISSION,
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
      type: ServiceKey.CONFIGURE_MISSION,
      response: {
        state: success ? 'success' : 'error',
        msg: `Config mission to ${deviceName} ${success ? 'ok' : 'fail'}`,
      },
    };
  }
  return null;
}
