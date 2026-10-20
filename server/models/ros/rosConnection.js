import * as ROSLIB from 'roslib';
import { logHelpers } from '../../common/logger.js';
import { ROS_RECONNECT_INTERVAL_MS, ROS_URL } from '../../config/config.js';

var ros = null;
const rosState = { state: 'disconnect', msg: 'init msg' };
let rosVersionInfo = { version: 1, distro: null };

var autoconectRos = null;
var noTimerflag = true;

// Holds the disconnect handler injected by the facade (unsubscribeDevice + GCSunServicesMission)
var _onDisconnect = null;
// Holds the connected handler injected by the facade (connectAllUAV + GCSServicesMission)
var _onConnected = null;

// rosbridge parses what it receives with ujson/simplejson/json, which all read the bare `NaN`
// literal; JSON.stringify would write null, which rosbridge rejects for a float field. PX4 reads
// NaN as "unset" (DO_GIMBAL_MANAGER_PITCHYAW rates), so it has to arrive as NaN.
const NAN_TOKEN = '__ros_nan__';
export function stringifyKeepingNaN(value) {
  return JSON.stringify(value, (_key, v) => (Number.isNaN(v) ? NAN_TOKEN : v)).replaceAll(`"${NAN_TOKEN}"`, 'NaN');
}

async function transportKeepingNaN(url) {
  const transport = await ROSLIB.WebSocketTransportFactory(url);
  // Both roslib transports (ws and native WebSocket) send JSON.stringify(message) over `socket`.
  transport.send = (message) => transport.socket.send(stringifyKeepingNaN(message));
  return transport;
}

export function getRos() {
  return ros;
}

export function setRosState({ state, msg }) {
  rosState['state'] = state;
  rosState['msg'] = msg;
}

export function serverStatus() {
  return rosState;
}

export function getRosVersionInfo() {
  return rosVersionInfo;
}

export function setRosVersionInfo({ version, distro }) {
  if (!version) {
    throw new Error('Invalid version');
  }
  rosVersionInfo = { version, distro: distro ?? null };
}

function _connectRos() {
  if (rosState.state != 'connect') {
    rosConnect(_onConnected);
  } else {
    clearInterval(autoconectRos);
    noTimerflag = true;
  }
}

function _autoConectRos() {
  if (noTimerflag) {
    noTimerflag = false;
    autoconectRos = setInterval(_connectRos, ROS_RECONNECT_INTERVAL_MS);
  }
}

function _disconectInternal() {
  if (_onDisconnect) {
    _onDisconnect();
  }
  _autoConectRos();
}

export function disconectRos(onDisconnect) {
  _onDisconnect = onDisconnect || _onDisconnect;
  _disconectInternal();
}

export function rosConnect(onConnected) {
  if (onConnected) {
    _onConnected = onConnected;
  }
  if (rosState.state != 'connect') {
    ros = new ROSLIB.Ros({ url: ROS_URL, encoding: 'utf8', transportFactory: transportKeepingNaN });
    ros.on('connection', function () {
      logHelpers.ros.connect('server', { status: 'connected' });
      setRosState({ state: 'connect', msg: 'Conected to ROS' });
      if (_onConnected) {
        _onConnected();
      }
    });
    ros.on('error', function (error) {
      logHelpers.ros.error('Error connect to server', error);
      setRosState({ state: 'error', msg: 'No se ha podido conectar a ROS' });
      _disconectInternal();
    });
    ros.on('close', function () {
      logHelpers.ros.error('Connection closed.', { message: 'Connection closed' });
      setRosState({ state: 'disconnect', msg: 'offline a ROS' });
      _disconectInternal();
    });
  }
}

export function initAutoConnect(onConnected, onDisconnect) {
  _onConnected = onConnected;
  if (onDisconnect) {
    _onDisconnect = onDisconnect;
  }
  _autoConectRos();
}
