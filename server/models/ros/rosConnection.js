import * as ROSLIB from 'roslib';
import { logHelpers } from '../../common/logger.js';
import { ROS_RECONNECT_INTERVAL_MS, ROS_URL } from '../../config/config.js';

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

/**
 * The rosbridge connection: client, state, reconnect timer and the handlers the facade injects.
 * A class (not module variables) so the state has one owner and a test can build its own.
 */
export class RosConnection {
  constructor() {
    this.ros = null;
    this.state = { state: 'disconnect', msg: 'init msg' };
    this.versionInfo = { version: 1, distro: null };
    this._reconnectTimer = null;
    // Injected by the facade: unsubscribeDevice + GCSunServicesMission / connectAllUAV + GCSServicesMission
    this._onDisconnect = null;
    this._onConnected = null;
  }

  setState({ state, msg }) {
    this.state['state'] = state;
    this.state['msg'] = msg;
  }

  setVersionInfo({ version, distro }) {
    if (!version) {
      throw new Error('Invalid version');
    }
    this.versionInfo = { version, distro: distro ?? null };
  }

  _tryReconnect() {
    if (this.state.state != 'connect') {
      this.connect(this._onConnected);
    } else {
      this._stopReconnect();
    }
  }

  _startReconnect() {
    if (!this._reconnectTimer) {
      this._reconnectTimer = setInterval(() => this._tryReconnect(), ROS_RECONNECT_INTERVAL_MS);
    }
  }

  _stopReconnect() {
    clearInterval(this._reconnectTimer);
    this._reconnectTimer = null;
  }

  _handleDisconnect({ notify = true } = {}) {
    if (notify && this._onDisconnect) {
      this._onDisconnect();
    }
    this._startReconnect();
  }

  disconnect(onDisconnect) {
    this._onDisconnect = onDisconnect || this._onDisconnect;
    this._handleDisconnect();
  }

  connect(onConnected) {
    if (onConnected) {
      this._onConnected = onConnected;
    }
    if (this.state.state != 'connect') {
      const client = new ROSLIB.Ros({ url: ROS_URL, encoding: 'utf8', transportFactory: transportKeepingNaN });
      this.ros = client;
      // A failed attempt fires `error` AND `close`, and every retry builds a new client. The
      // disconnect handler undoes what a connection set up (subscriptions, GCS services), so it
      // runs once per connection that was really established and then lost — not per event, and
      // not for attempts that never connected.
      let established = false;
      let lost = false;
      const onLost = () => {
        // Late events from a client already replaced by a retry must not touch the live state.
        if (client !== this.ros) return;
        this._handleDisconnect({ notify: established && !lost });
        lost = true;
      };
      client.on('connection', () => {
        if (client !== this.ros) return;
        established = true;
        lost = false;
        logHelpers.ros.connect('server', { status: 'connected' });
        this.setState({ state: 'connect', msg: 'Conected to ROS' });
        if (this._onConnected) {
          this._onConnected();
        }
      });
      client.on('error', (error) => {
        if (client !== this.ros) return;
        logHelpers.ros.error('Error connect to server', error);
        this.setState({ state: 'error', msg: 'No se ha podido conectar a ROS' });
        onLost();
      });
      client.on('close', () => {
        if (client !== this.ros) return;
        logHelpers.ros.error('Connection closed.', { message: 'Connection closed' });
        this.setState({ state: 'disconnect', msg: 'offline a ROS' });
        onLost();
      });
    }
  }

  initAutoConnect(onConnected, onDisconnect) {
    this._onConnected = onConnected;
    if (onDisconnect) {
      this._onDisconnect = onDisconnect;
    }
    this._startReconnect();
  }
}

const connection = new RosConnection();

export const getRos = () => connection.ros;
export const setRosState = (state) => connection.setState(state);
export const serverStatus = () => connection.state;
export const getRosVersionInfo = () => connection.versionInfo;
export const setRosVersionInfo = (info) => connection.setVersionInfo(info);
export const disconectRos = (onDisconnect) => connection.disconnect(onDisconnect);
export const rosConnect = (onConnected) => connection.connect(onConnected);
export const initAutoConnect = (onConnected, onDisconnect) => connection.initAutoConnect(onConnected, onDisconnect);
