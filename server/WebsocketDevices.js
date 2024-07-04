//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"
import * as fb from './dist/schema_main.cjs';
import * as flatbuffers from 'flatbuffers'; // do not remove; needed by generated code
import WebSocket, { WebSocketServer } from 'ws';
import { DevicesController } from './controllers/devices.js';
import { positionsController } from './controllers/positions.js';
import { parse } from 'url';
import { readYAML, getDatetime } from './common/utils.js';

const devices_msg = readYAML('../config/devices/devices_msg.yaml');

function getRobofleetMetadata(buf) {
  const msg = fb.fb.MsgWithMetadata.getRootAsMsgWithMetadata(buf);
  return msg._Metadata();
}

export function getByteBuffer(data) {
  if (data instanceof Buffer) {
    return new flatbuffers.ByteBuffer(data);
  }
  if (data instanceof ArrayBuffer) {
    return new flatbuffers.ByteBuffer(new Uint8Array(data));
  }
  return null;
}
function getNameFromTopic(topic) {
  // Absolute vs relative path
  if (topic.startsWith('/')) {
    return topic.split('/')[1];
  } else {
    return topic.split('/')[0];
  }
}

function encodePositionStanpedd({ namespace, topic, frame = 'map', x, y, theta }) {
  const fbb = new flatbuffers.Builder();

  const metadataOffset = fb.fb.MsgMetadata.createMsgMetadata(
    fbb,
    fbb.createString('geometry_msgs/PoseStamped'),
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
function encodeMission({ topic, attributes }) {
  const fbb = new flatbuffers.Builder();
  const metadataOffset = fb.fb.MsgMetadata.createMsgMetadata(
    fbb,
    fbb.createString('mission_planner/Mission'),
    fbb.createString(`${topic}`)
  );
  const frame = 'map';
  const uavIdOffset = fbb.createString(frame);
  const missionId = fbb.createString(frame);
  const missionType = fbb.createString(frame);

  const frameOffset = fbb.createString(frame);

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

  const waypointsOffset = fb.fb.mission_msgs.MissionRequest.createWaypointVector(fbb, waypoints);
  const yawOffset = fb.fb.mission_msgs.MissionRequest.createYawVector(fbb, attributes.yaw);
  const GimbalPitch = fb.fb.mission_msgs.MissionRequest.createGimbalPitchVector(fbb, attributes.gimbalPitch);
  const speedOffset = fb.fb.mission_msgs.MissionRequest.createSpeedVector(fbb, attributes.speed);
  const commandListOffset = fb.fb.mission_msgs.MissionRequest.createCommandListVector(fbb, attributes.commandList);
  const commandParameterOffset = fb.fb.mission_msgs.MissionRequest.createCommandParameterVector(
    fbb,
    attributes.commandParameter
  );

  fb.fb.mission_msgs.MissionRequest.startMissionRequest(fbb);
  fb.fb.mission_msgs.MissionRequest.add_Metadata(fbb, metadataOffset);
  fb.fb.mission_msgs.MissionRequest.addUavId(fbb, uavIdOffset);
  fb.fb.mission_msgs.MissionRequest.addMissionId(fbb, missionId);
  fb.fb.mission_msgs.MissionRequest.addMissionType(fbb, missionType);
  fb.fb.mission_msgs.MissionRequest.addWaypoint(fbb, waypointsOffset);
  fb.fb.mission_msgs.MissionRequest.addRadius(fbb, attributes.radius);
  fb.fb.mission_msgs.MissionRequest.addMaxVel(fbb, attributes.maxVel);
  fb.fb.mission_msgs.MissionRequest.addIdleVel(fbb, attributes.idleVel);
  fb.fb.mission_msgs.MissionRequest.addYaw(fbb, yawOffset);
  fb.fb.mission_msgs.MissionRequest.addGimbalPitch(fbb, GimbalPitch);
  fb.fb.mission_msgs.MissionRequest.addSpeed(fbb, speedOffset);
  fb.fb.mission_msgs.MissionRequest.addYawMode(fbb, attributes.yawMode);
  fb.fb.mission_msgs.MissionRequest.addTraceMode(fbb, attributes.traceMode);
  fb.fb.mission_msgs.MissionRequest.addGimbalPitchMode(fbb, attributes.gimbalPitchMode);
  fb.fb.mission_msgs.MissionRequest.addFinishAction(fbb, attributes.finishAction);
  fb.fb.mission_msgs.MissionRequest.addCommandList(fbb, commandListOffset);
  fb.fb.mission_msgs.MissionRequest.addCommandParameter(fbb, commandParameterOffset);

  const MissionOffset = fb.fb.mission_msgs.MissionRequest.endMissionRequest(fbb);

  fbb.finish(MissionOffset);
  return fbb.asUint8Array();
}

function encodeCommandMission({ topic }) {
  console.log('encodeCommandMission');
  const fbb = new flatbuffers.Builder();
  const metadataOffset = fb.fb.MsgMetadata.createMsgMetadata(
    fbb,
    fbb.createString('mission_planner/Mission'),
    fbb.createString(`${topic}`)
  );
  fbb.finish(metadataOffset);
  return fbb.asUint8Array();
}

async function encode({ uav_id, type, attributes }) {
  let device = await DevicesController.getDevice(uav_id);
  let uavName = device.name;
  let uavCategory = device.category;

  if (!devices_msg[uavCategory]['services'].hasOwnProperty(type)) {
    console.log(type + ' to:' + uavName + ' dont have this service');
    return { state: 'warning', msg: type + ' to:' + uavName + ' dont have this service' };
  }

  let topic = uavName + devices_msg[uavCategory]['services'][type]['name'];
  let serviceType = devices_msg[uavCategory]['services'][type]['serviceType'];

  if (type === 'position') {
    let x = attributes.x;
    let y = attributes.y;
    let theta = attributes.theta;
    return encodePositionStanpedd({ topic, frame: 'map', x, y, theta });
  }
  if (type === 'configureMission') {
    return encodeMission({ topic, attributes });
  }
  if (type === 'commandMission') {
    return encodeCommandMission({ topic });
  }
}

async function decoder(metadata, buf, name) {
  let deviceName = name;
  console.log('name of device:' + name);
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
  }
  if (metadata.type() === 'sensor_msgs/Imu') {
    const msg = fb.fb.geometry_msgs.Vector3.getRootAsVector3(buf);
    positionsController.updatePosition({ deviceId, course: 90 + msg.z() * 57.295 });
  }
  if (metadata.type() === 'sensor_msgs/BatteryState') {
    const msg = fb.fb.sensor_msgs.BatteryState.getRootAsBatteryState(buf);
    positionsController.updatePosition({
      deviceId,
      batteryLevel: (msg.percentage() * 100).toFixed(0),
    }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
  }
  if (metadata.type() === 'geometry_msgs/Vector3Stamped' && metadata.topic().includes('gimbal')) {
    const msg = fb.fb.geometry_msgs.Vector3Stamped.getRootAsVector3Stamped(buf);
    positionsController.updatePosition({
      deviceId,
      gimbal: { x: msg.vector().x(), y: msg.vector().y(), z: msg.vector().z() },
    }); //  showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
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
  }
}

function heartbeat() {
  this.isAlive = true;
}

export async function sendCommandToClient({ uav_id, type, attributes }) {
  let nameClient = await DevicesController.getDevice(uav_id);
  console.log('send command to ' + nameClient.name);
  let msg = await encode({ uav_id, type, attributes });
  WebsocketDevices._instance.sentToClient(nameClient.name, msg);
}

export class WebsocketDevices {
  constructor(port) {
    this.clients = new Map();
    console.log('init websocket manager' + WebsocketDevices._instance);
    if (WebsocketDevices._instance) {
      console.log('return last instance singleton');
      return WebsocketDevices._instance;
    }
    console.log('new instance create');
    WebsocketDevices._instance = this;

    this.ws = new WebSocket.Server({ port: port });

    this.onConnect(async (client) => {
      let init_msg = 'Wellcome new device';
      client.notify(init_msg);

      client.onMessage(async (message, name) => {
        const buf = getByteBuffer(message);
        if (buf === null) {
          console.log("received message that wasn't a flatbuffer. ignoring.");
          return;
        }
        const metadata = getRobofleetMetadata(buf) ?? null;

        if (metadata?.topic === null) {
          console.log('received message without topic. ignoring.');
          return;
        }
        //console.log('received message on topic: %s', metadata.topic());
        //console.log('received message type    : %s', metadata.type());
        decoder(metadata, buf, name);
      });
    });

    const interval = setInterval(this.ping.bind(this), 30000);

    this.ws.on('close', function close() {
      clearInterval(interval);
    });
  }

  ping() {
    this._applyToAllClients((client) => {
      if (client.isAlive === false) return client.terminate();
      client.isAlive = false;
      client.ping();
    });
  }

  broadcast(message) {
    this._applyToAllClients((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  }

  broadcastToRoom(room, message) {
    const clients = this.clientsToRoom.get(room) || [];
    clients.forEach((client) => {
      client.notify(message);
    });
  }

  sentToClient(nameClient, message) {
    const client = this.clients.get(nameClient);
    if (client) {
      client.notify(message);
    }
    //this.clients.forEach((client) => {
    //  if (client.name === nameClient) {
    //    client.notify(message);
    //  }
    //});
  }

  disconnect() {
    this.ws.close();
  }

  _applyToAllClients(cb) {
    this.ws.clients.forEach(cb);
  }

  onConnect(cb) {
    this.ws.on('connection', (client, request) => {
      console.log(request.url);
      const name = parse(request.url, true).query['name'] || null;
      console.log('name:' + name);

      client.isAlive = true;

      console.log('New device is connected');

      client.on('error', console.error);

      client.on('pong', heartbeat);

      client.on('close', () => {
        console.log('device Connection closed');
      });
      let myWebsocketClient = new WebsocketClient(this, { client, name });
      this.clients.set(name, myWebsocketClient);

      cb(myWebsocketClient);
    });
  }
}

export class WebsocketClient {
  constructor(wsManager, { client, name }) {
    this.client = client;
    this.wsManager = wsManager;
    this.name = name;
  }

  notify(message) {
    if (this.client.readyState === WebSocket.OPEN) {
      this.client.send(message);
    }
  }

  onMessage(cb) {
    this.client.on('message', (message) => {
      cb(message, this.name);
    });
  }
}
