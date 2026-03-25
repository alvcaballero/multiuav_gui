import * as fb from 'fbmsglib';
import * as flatbuffers from 'flatbuffers';
import { devicesController } from '../../controllers/devices.js';
import { readDataFile } from '../../common/utils.js';

const devices_msg = readDataFile('../config/devices/devices_msg.yaml');

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

function encodeMission({ topic, type, attributes }) {
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
  const yawOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createYawVector(fbb, attributes.yaw);
  const gimbalPitchOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createGimbalPitchVector(fbb, attributes.gimbalPitch);
  const speedOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createSpeedVector(fbb, attributes.speed);
  const commandListOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createCommandListVector(
    fbb,
    attributes.commandList.flat()
  );
  const commandParameterOffset = fb.fb.aerialcore_common.ConfigMissionRequest.createCommandParameterVector(
    fbb,
    attributes.commandParameter.flat()
  );

  fb.fb.aerialcore_common.ConfigMissionRequest.startConfigMissionRequest(fbb);
  fb.fb.aerialcore_common.ConfigMissionRequest.addUavId(fbb, uavIdOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addMissionId(fbb, missionIdOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addMissionType(fbb, attributes.missionType ?? 0);
  fb.fb.aerialcore_common.ConfigMissionRequest.addWaypoint(fbb, waypointsOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addRadius(fbb, attributes.radius);
  fb.fb.aerialcore_common.ConfigMissionRequest.addMaxVel(fbb, attributes.maxVel);
  fb.fb.aerialcore_common.ConfigMissionRequest.addIdleVel(fbb, attributes.idleVel);
  fb.fb.aerialcore_common.ConfigMissionRequest.addYaw(fbb, yawOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addGimbalPitch(fbb, gimbalPitchOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addSpeed(fbb, speedOffset);
  fb.fb.aerialcore_common.ConfigMissionRequest.addYawMode(fbb, attributes.yawMode);
  fb.fb.aerialcore_common.ConfigMissionRequest.addTraceMode(fbb, attributes.traceMode);
  fb.fb.aerialcore_common.ConfigMissionRequest.addGimbalPitchMode(fbb, attributes.gimbalPitchMode);
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

function encodeCommandMission({ topic, type, attributes }) {
  const fbb = new flatbuffers.Builder();
  const metadataOffset = fb.fb.MsgMetadata.createMsgMetadata(
    fbb,
    fbb.createString(`${type}`),
    fbb.createString(`${topic}`)
  );
  const requestOffset = fb.fb.std_srvs.SetBoolRequest.createSetBoolRequest(fbb, true);

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

  if (!devices_msg[uavCategory]['services'].hasOwnProperty(type)) {
    return null;
  }

  const serviceType = devices_msg[uavCategory]['services'][type]['serviceType'];

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
  if (type === 'configureMission') {
    return encodeMission({ topic: `/${uavName}/configureMission`, type: serviceType, attributes });
  }
  if (type === 'commandMission') {
    return encodeCommandMission({ topic: `/${uavName}/commandMission`, type: serviceType, attributes });
  }

  return null;
}
