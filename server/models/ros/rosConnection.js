import * as ROSLIB from 'roslib';
import logger, { logHelpers } from '../../common/logger.js';
import { ROS_RECONNECT_INTERVAL_MS, ROS_URL } from '../../config/config.js';

var ros = null;
const rosState = { state: 'disconnect', msg: 'init msg' };

var autoconectRos = null;
var noTimerflag = true;

logHelpers.system.info('roslib version:', ROSLIB.version || 'unknown');

// Holds the disconnect handler injected by the facade (unsubscribeDevice + GCSunServicesMission)
var _onDisconnect = null;
// Holds the connected handler injected by the facade (connectAllUAV + GCSServicesMission)
var _onConnected = null;

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
    ros = new ROSLIB.Ros({ url: ROS_URL, encoding: 'utf8' });
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
