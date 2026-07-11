import * as ROSLIB from 'roslib';
import { readDataFile } from '../../common/utils.js';
import { logger } from '../../common/logger.js';
import { getActionServer } from './rosInspect.js';
import { encodeRosSrv } from './rosEncode.js';

const devices_msg = readDataFile('../config/devices/devices_msg.yaml');

/**
 * Resolve a device action from the config into the ROS action-server name and
 * type. Shared by send/cancel/status so they all key the registry identically.
 *
 * @param {string} name     - Device name, e.g. agv_1
 * @param {string} category - Device category, e.g. agv
 * @param {string} type     - Action key in devices_msg[category].actions, e.g. navigateToPose
 * @returns {{ actionServerName: string, actionType: string }}
 * @throws if the category has no such action
 */
function resolveDeviceAction(name, category, type) {
  const actions = devices_msg[category]?.actions;
  if (!actions || !actions.hasOwnProperty(type)) {
    throw new Error(`Action '${type}' not configured for category '${category}' (device ${name})`);
  }
  const cfg = actions[type];
  return { actionServerName: `/${name}${cfg.name}`, actionType: cfg.actionType };
}

export const ActionStatus = Object.freeze({
  EXECUTING: 'executing',
  SUCCEEDED: 'succeeded',
  ABORTED: 'aborted',
  CANCELING: 'canceling',
  CANCELED: 'canceled',
  IDLE: 'idle',
});

/**
 * Registry of active ROS actions keyed by action name (e.g. /bcr_bot_1/navigate_to_pose).
 * One slot per action server — a new goal replaces the previous entry.
 *
 * Entry shape:
 * {
 *   client:    ROSLIB.Action,
 *   goalId:    string,
 *   status:    ActionStatus,
 *   target:    object,       // original goal args for status queries
 *   startedAt: Date,
 *   endedAt:   Date | null,
 *   msg:       string,
 * }
 */
const registry = new Map();

// ─────────────────────────────────────────────────────────────────────────
// Primitives — operate on already-resolved ROS action names/types, never on
// device config. Parallel to callRosService in rosServices.js.
// ─────────────────────────────────────────────────────────────────────────

/**
 * Sends a ROS action goal and registers it under the action-server name.
 * Pure ROS-transport primitive: receives the fully-resolved server name and
 * type, knows nothing about devices_msg.
 *
 * @param {object} args
 * @param {string} args.actionServerName - Full action server name, e.g. /agv_1/navigate_to_pose
 * @param {string} args.actionType       - e.g. nav2_msgs/action/NavigateToPose
 * @param {object} args.message          - Goal message
 * @param {object} [args.target]         - Human-readable goal summary stored for status queries
 * @param {number} [args.timeout]        - ms, default 120 000 (only applies when blocking=true)
 * @param {boolean} [args.blocking]      - If true, waits until the action completes before resolving
 * @param {function} [args.onComplete]   - Called once with the final {state, msg, goalId} when the
 *                                          action reaches a terminal state (succeeded/aborted), regardless
 *                                          of blocking. Use this for non-blocking goals when the caller
 *                                          isn't awaiting the returned promise.
 * @param {object} ros                   - Connected ROSLIB.Ros instance
 * @returns {Promise<{state, msg, goalId}>}
 */
export async function sendRosActionGoal(args, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const { actionServerName, actionType, message, target = {}, timeout = 120_000, blocking = false, onComplete } = args;

  if (!actionServerName) throw new Error('Missing required arg: actionServerName');
  if (!actionType) throw new Error('Missing required arg: actionType');
  if (!message || typeof message !== 'object') throw new Error('Missing required arg: message (must be an object)');

  let servers = await getActionServer(ros);
  if (!servers.toString().includes(actionServerName)) {
    throw new Error(
      `Action server ${actionServerName} not found (posible robot is no available or not running expected software)`
    );
  }
  const client = new ROSLIB.Action({ ros, name: actionServerName, actionType });

  // Cancel any previous active goal on this action server before registering a new one
  const previous = registry.get(actionServerName);
  if (previous && previous.status === ActionStatus.EXECUTING) {
    logger.warn(`[ActionRegistry] Replacing active goal on ${actionServerName}`);
    _cancelEntry(actionServerName, previous);
  }

  const promise = new Promise((resolve, reject) => {
    const timer = blocking
      ? setTimeout(() => {
          // The action is still running on Nav2 — keep status as EXECUTING.
          // Only the caller that was waiting gives up; the registry callback
          // will still update the entry when Nav2 eventually reports a result.
          reject(new Error(`Action goal timed out after ${timeout}ms — action may still be executing`));
        }, timeout)
      : null;

    const goalId = client.sendGoal(
      message,
      // Called by ROSLIB only when status === STATUS_SUCCEEDED (4); receives action result values
      (result) => {
        if (timer) clearTimeout(timer);
        logger.info(`[ActionRegistry] Result on ${actionServerName}: ${JSON.stringify(result)}`);

        const entry = registry.get(actionServerName);
        if (entry) {
          entry.status = ActionStatus.SUCCEEDED;
          entry.endedAt = new Date();
          entry.msg = 'Action completed';
        }

        const actionResult = {
          state: 'success',
          msg: entry?.msg ?? 'Action completed',
          goalId,
        };
        resolve(actionResult);
        _notifyComplete(onComplete, actionServerName, actionResult);
      },
      (feedback) => {
        logger.debug(`[ActionRegistry] Feedback on ${actionServerName}: ${JSON.stringify(feedback)}`);
      },
      // Called by ROSLIB when status !== STATUS_SUCCEEDED (aborted, canceled, etc.)
      (error) => {
        if (timer) clearTimeout(timer);
        logger.error(`[ActionRegistry] Error on ${actionServerName}: ${error}`);

        const entry = registry.get(actionServerName);
        if (entry) {
          entry.status = ActionStatus.ABORTED;
          entry.endedAt = new Date();
          entry.msg = String(error);
        }

        // Nav2 abort/cancel is a valid action result, not a server error — resolve with state
        const result = { state: 'aborted', msg: String(error), code: error.code, goalId };
        resolve(result);
        _notifyComplete(onComplete, actionServerName, result);
      }
    );

    logger.info(`[ActionRegistry] Goal sent on ${actionServerName} — ID: ${goalId} | blocking: ${blocking}`);

    registry.set(actionServerName, {
      client,
      goalId,
      status: ActionStatus.EXECUTING,
      target,
      startedAt: new Date(),
      endedAt: null,
      msg: 'Executing',
    });

    if (!blocking) {
      resolve({ state: 'goal_sent', msg: `Goal sent to ${actionServerName}`, goalId });
    }
  });

  return promise;
}

// Invokes the caller's onComplete without letting it crash the ROS event handler.
function _notifyComplete(onComplete, actionServerName, result) {
  if (!onComplete) return;
  try {
    onComplete(result);
  } catch (e) {
    logger.warn(`[ActionRegistry] onComplete callback threw for ${actionServerName}: ${e.message}`);
  }
}

// Serializes a registry entry into the public status shape.
function _entryStatus(entry) {
  return {
    status: entry.status,
    goalId: entry.goalId,
    target: entry.target,
    startedAt: entry.startedAt,
    endedAt: entry.endedAt,
    msg: entry.msg,
  };
}

/**
 * Reads action status from the registry by raw key/prefix.
 *
 * @param {object} params
 * @param {string} [params.actionServerName] - Exact registry key, e.g. /agv_1/navigate_to_pose
 * @param {string} [params.prefix]           - Return all entries whose key starts with this
 */
export function getRosActionStatus({ actionServerName, prefix } = {}) {
  // prefix → all entries under a device
  if (prefix) {
    const results = {};
    for (const [key, entry] of registry) {
      if (key.startsWith(prefix)) results[key] = _entryStatus(entry);
    }
    return Object.keys(results).length
      ? results
      : { status: ActionStatus.IDLE, msg: `No actions registered for ${prefix}` };
  }

  const entry = registry.get(actionServerName);
  if (!entry) return { status: ActionStatus.IDLE, msg: 'No action registered' };
  return _entryStatus(entry);
}

/**
 * Cancels the active goal for an action server by raw key (no-op if idle or
 * already finished).
 *
 * @param {object} params
 * @param {string} params.actionServerName - Exact registry key
 */
export function cancelRosAction({ actionServerName } = {}) {
  if (!actionServerName) return { state: 'error', msg: 'Missing required param: actionServerName' };

  const entry = registry.get(actionServerName);
  if (!entry) return { state: 'warning', msg: `No action registered for ${actionServerName}` };
  if (entry.status !== ActionStatus.EXECUTING) {
    return { state: 'warning', msg: `Action ${actionServerName} is not executing (status: ${entry.status})` };
  }

  _cancelEntry(actionServerName, entry);
  return { state: 'success', msg: `Goal canceled for ${actionServerName}` };
}

function _cancelEntry(key, entry) {
  try {
    entry.client.cancel(entry.goalId);
  } catch (e) {
    logger.warn(`[ActionRegistry] Error canceling goal on ${key}: ${e.message}`);
  }
  entry.status = ActionStatus.CANCELING;
  entry.endedAt = new Date();
  entry.msg = 'Canceled by operator';
}

// ─────────────────────────────────────────────────────────────────────────
// Device layer — resolve the action from devices_msg config, then delegate to
// the primitives above. Parallel to callService in rosServices.js.
// ─────────────────────────────────────────────────────────────────────────

/**
 * Sends an action goal to a device, resolving the ROS name/type from config.
 *
 * @param {object} args
 * @param {string} args.name     - Device name, e.g. agv_1
 * @param {string} args.category - Device category, e.g. agv
 * @param {string} args.type     - Action config key, e.g. navigateToPose
 * @param {object} args.message  - Goal message
 * @param {object} [args.target]
 * @param {number} [args.timeout]
 * @param {boolean} [args.blocking]
 * @param {function} [args.onComplete] - See sendRosActionGoal
 * @param {object} ros
 */
export async function sendActionGoal({ name, category, type, message, ...rest }, ros) {
  const { actionServerName, actionType } = resolveDeviceAction(name, category, type);
  const goalMessage = encodeRosSrv({ type, msg: message, msgType: actionType });
  return sendRosActionGoal({ actionServerName, actionType, message: goalMessage, ...rest }, ros);
}

/**
 * Reads action status for a device.
 *  - { name, category }       → all actions registered for that device
 *  - { name, category, type } → status of that specific action
 */
export function getActionStatus({ name, category, type } = {}) {
  if (name && !type) {
    return getRosActionStatus({ prefix: `/${name}/` });
  }
  const { actionServerName } = resolveDeviceAction(name, category, type);
  return getRosActionStatus({ actionServerName });
}

/**
 * Cancels a device's active action goal.
 */
export function cancelAction({ name, category, type } = {}) {
  const { actionServerName } = resolveDeviceAction(name, category, type);
  return cancelRosAction({ actionServerName });
}
