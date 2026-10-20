import * as fb from 'fbmsglib';
import * as flatbuffers from 'flatbuffers';
import { devicesController } from '../../controllers/devices.js';
import { categoryModel } from '../category.js';
import { ServiceKey } from '../../config/deviceMsgCatalog.js';

function encodePositionStamped({ topic, type = 'geometry_msgs/PoseStamped', frame = 'map', x, y, theta }) {
  const fbb = new flatbuffers.Builder();

  const metadataOffset = fb.fb.MsgMetadata.createMsgMetadata(
    fbb,
    fbb.createString(`${type}`),
    fbb.createString(`${topic}`)
  );

  const frameOffset = fbb.createString(frame);

  fb.fb.std_msgs.Header.startHeader(fbb);
  fb.fb.std_msgs.Header.addStamp(fbb, fb.fb.RosTime.createRosTime(fbb, Math.floor(Date.now() / 1000), 0));
  fb.fb.std_msgs.Header.addFrameId(fbb, frameOffset);
  const headerOffset = fb.fb.std_msgs.Header.endHeader(fbb);

  const positionOffset = fb.fb.geometry_msgs.Vector3.createVector3(fbb, 0, x, y, 0);
  const z = Math.sin(theta / 2);
  const w = Math.cos(theta / 2);
  const orientationOffset = fb.fb.geometry_msgs.Quaternion.createQuaternion(fbb, 0, 0, 0, z, w);

  fb.fb.geometry_msgs.Pose.startPose(fbb);
  fb.fb.geometry_msgs.Pose.addPosition(fbb, positionOffset);
  fb.fb.geometry_msgs.Pose.addOrientation(fbb, orientationOffset);
  const poseOffset = fb.fb.geometry_msgs.Pose.endPose(fbb);

  fb.fb.geometry_msgs.PoseStamped.startPoseStamped(fbb);
  fb.fb.geometry_msgs.PoseStamped.add_Metadata(fbb, metadataOffset);
  fb.fb.geometry_msgs.PoseStamped.addHeader(fbb, headerOffset);
  fb.fb.geometry_msgs.PoseStamped.addPose(fbb, poseOffset);
  const stampedOffset = fb.fb.geometry_msgs.PoseStamped.endPoseStamped(fbb);

  fbb.finish(stampedOffset);
  return fbb.asUint8Array();
}

// The ROS message arrives already encoded for its msgType (models/msgEncode.js, before the
// gateway): robofleet packs ROS messages, so this module only PACKS them into the Flatbuffer
// schema generated from the same .srv — it never builds a message from domain data.

// std_msgs/*MultiArray in rosbridge JSON is `{ data: [...] }`; the Flatbuffer schema stores
// the plain vector.
const dataOf = (field) => field?.data ?? [];

// serviceTypes whose ROS message (MissionToRos) matches the generated
// fb.aerialcore_common.ConfigMissionRequest schema.
const CONFIG_MISSION_TYPES = new Set(['aerialcore_common/ConfigMission', 'multiuav_interfaces/ConfigMission']);

/**
 * Packs an aerialcore_common/ConfigMission request (the output of MissionToRos) into Flatbuffers.
 * @param {object} msg  ROS ConfigMission request: waypoint[], radius, maxVel, idleVel,
 *                      yaw/speed/gimbalPitch/commandList/commandParameter as { data: [] }, modes
 * @param {{ topic: string, type: string }} meta  topic and serviceType for MsgMetadata
 * @returns {Uint8Array}
 */
export function encodeConfigMission(msg, { topic, type }) {
  const attributes = msg;
  const fbb = new flatbuffers.Builder();
  const metadataOffset = fb.fb.MsgMetadata.createMsgMetadata(
    fbb,
    fbb.createString(`${type}`),
    fbb.createString(`${topic}`)
  );
  // FIXED: were all hardcoded to 'map'; now use actual attributes values
  // const uavIdOffset = fbb.createString(frame);
  // const missionId   = fbb.createString(frame);
  // const missionType = fbb.createString(frame);  // also wrong type: schema is uint8, not string
  const uavIdOffset = fbb.createString(attributes.uavId ?? '');
  const missionIdOffset = fbb.createString(String(attributes.missionId ?? ''));
  const frameOffset = fbb.createString('map');

  fb.fb.std_msgs.Header.startHeader(fbb);
  fb.fb.std_msgs.Header.addStamp(fbb, fb.fb.RosTime.createRosTime(fbb, Math.floor(Date.now() / 1000), 0));
  fb.fb.std_msgs.Header.addFrameId(fbb, frameOffset);
  const headerOffset = fb.fb.std_msgs.Header.endHeader(fbb);

  fb.fb.sensor_msgs.NavSatStatus.startNavSatStatus(fbb);
  fb.fb.sensor_msgs.NavSatStatus.addStatus(fbb, 0);
  fb.fb.sensor_msgs.NavSatStatus.addService(fbb, 0);
  const navSatStatusOffset = fb.fb.sensor_msgs.NavSatStatus.endNavSatStatus(fbb);

  let waypoints = [];
  for (const waypoint of attributes.waypoint) {
    fb.fb.sensor_msgs.NavSatFix.startNavSatFix(fbb);
    fb.fb.sensor_msgs.NavSatFix.add_Metadata(fbb, metadataOffset);
    fb.fb.sensor_msgs.NavSatFix.addHeader(fbb, headerOffset);
    fb.fb.sensor_msgs.NavSatFix.addStatus(fbb, navSatStatusOffset);
    fb.fb.sensor_msgs.NavSatFix.addLatitude(fbb, waypoint.latitude);
    fb.fb.sensor_msgs.NavSatFix.addLongitude(fbb, waypoint.longitude);
    fb.fb.sensor_msgs.NavSatFix.addAltitude(fbb, waypoint.altitude);
    fb.fb.sensor_msgs.NavSatFix.addPositionCovariance(fbb, [0, 0, 0, 0, 0, 0, 0, 0, 0]);
    waypoints.push(fb.fb.sensor_msgs.NavSatFix.endNavSatFix(fbb));
  }

  const waypointsOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createWaypointVector(fbb, waypoints);
  const yawOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createYawVector(fbb, dataOf(attributes.yaw));
  const gimbalPitchOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createGimbalPitchVector(
    fbb,
    dataOf(attributes.gimbalPitch)
  );
  const speedOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createSpeedVector(fbb, dataOf(attributes.speed));
  const commandListOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createCommandListVector(
    fbb,
    dataOf(attributes.commandList)
  );
  const commandParameterOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createCommandParameterVector(
    fbb,
    dataOf(attributes.commandParameter)
  );

  fb.fb.aerialcore_common.ConfigMissionRequest.startConfigMissionRequest(fbb);
  fb.fb.aerialcore_common.ConfigMissionRequest.addUavId(fbb, uavIdOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addMissionId(fbb, missionIdOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addMissionType(fbb, attributes.missionType ?? 0);
  fb.fb.aerialcore_common.ConfigMissionRequest.addWaypoint(fbb, waypointsOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addRadius(fbb, attributes.radius ?? 0);
  // maxVel / gimbalPitchMode are omitted on profiles that don't expose them (v1): pack 0.
  fb.fb.aerialcore_common.ConfigMissionRequest.addMaxVel(fbb, attributes.maxVel ?? 0);
  fb.fb.aerialcore_common.ConfigMissionRequest.addIdleVel(fbb, attributes.idleVel);
  fb.fb.aerialcore_common.ConfigMissionRequest.addYaw(fbb, yawOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addGimbalPitch(fbb, gimbalPitchOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addSpeed(fbb, speedOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addYawMode(fbb, attributes.yawMode);
  fb.fb.aerialcore_common.ConfigMissionRequest.addTraceMode(fbb, attributes.traceMode);
  fb.fb.aerialcore_common.ConfigMissionRequest.addGimbalPitchMode(fbb, attributes.gimbalPitchMode ?? 0);
  fb.fb.aerialcore_common.ConfigMissionRequest.addFinishAction(fbb, attributes.finishAction);
  fb.fb.aerialcore_common.ConfigMissionRequest.addCommandList(fbb, commandListOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addCommandParameter(fbb, commandParameterOffset);
  const missionOffset = fb.fb.aerialcore_common.ConfigMissionRequest.endConfigMissionRequest(fbb);

  fb.fb.aerialcore_common.ConfigMission.startConfigMission(fbb);
  fb.fb.aerialcore_common.ConfigMission.add_Metadata(fbb, metadataOffset);
  fb.fb.aerialcore_common.ConfigMission.addRequest(fbb, missionOffset);
  const configMissionOffset = fb.fb.aerialcore_common.ConfigMission.endConfigMission(fbb);

  fbb.finish(configMissionOffset);
  return fbb.asUint8Array();
}

// command_mission request → std_srvs/SetBool Flatbuffer. `data` defaults to true (start): a
// category may declare command_mission as std_srvs/TriggerRequest, whose encoded request is {}.
export function encodeSetBool(msg, { topic, type }) {
  const fbb = new flatbuffers.Builder();
  const metadataOffset = fb.fb.MsgMetadata.createMsgMetadata(
    fbb,
    fbb.createString(`${type}`),
    fbb.createString(`${topic}`)
  );
  const requestOffset = fb.fb.std_srvs.SetBoolRequest.createSetBoolRequest(fbb, msg?.data ?? true);

  fb.fb.std_srvs.SetBool.startSetBool(fbb);
  fb.fb.std_srvs.SetBool.add_Metadata(fbb, metadataOffset);
  fb.fb.std_srvs.SetBool.addRequest(fbb, requestOffset);
  const setBoolOffset = fb.fb.std_srvs.SetBool.endSetBool(fbb);

  fbb.finish(setBoolOffset);
  return fbb.asUint8Array();
}

export async function encodeFbMsg({ uav_id, type, attributes }) {
  const device = await devicesController.getDevice(uav_id);
  const { name: uavName, category: uavCategory } = device;

  // categoryModel is the single source of truth for devices_msg — resolve the
  // service definition through it instead of reading the YAML here.
  const serviceDef = categoryModel.getCategory(uavCategory)?.services?.[type];
  if (!serviceDef) {
    return null;
  }

  const serviceType = serviceDef.messageType;

  if (type === 'position') {
    return encodePositionStamped({
      topic: `/${uavName}/initialpose`,
      type: 'geometry_msgs/PoseStamped',
      frame: 'map',
      x: attributes.x,
      y: attributes.y,
      theta: attributes.theta,
    });
  }
  // The topics keep robofleet's own names (/configureMission, /commandMission): the drone side
  // matches them, they are not the devices_msg keys.
  if (type === ServiceKey.CONFIGURE_MISSION) {
    // Only the ROS1 ConfigMission structure has a generated Flatbuffer schema; packing another
    // mission type (ROS2 LoadMission, PSDK) into it would send garbage.
    if (!CONFIG_MISSION_TYPES.has(serviceType)) {
      throw new Error(`robofleet has no Flatbuffer schema for mission type '${serviceType}' (device ${uavName})`);
    }
    return encodeConfigMission(attributes, { topic: `/${uavName}/configureMission`, type: serviceType });
  }
  if (type === ServiceKey.COMMAND_MISSION) {
    return encodeSetBool(attributes, { topic: `/${uavName}/commandMission`, type: serviceType });
  }

  return null;
}
