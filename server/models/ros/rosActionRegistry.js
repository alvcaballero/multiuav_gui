import * as ROSLIB from 'roslib';
import logger from '../../common/logger.js';
import { getActionServer } from './rosServices.js';

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

/**
 * Sends a ROS action goal and registers it under the action name.
 *
 * @param {object} args
 * @param {string} args.device    - Used for validating the action server exists, e.g. bcr_bot_1
 * @param {string} args.action      - Full action server name, e.g. /bcr_bot_1/navigate_to_pose
 * @param {string} args.actionType  - e.g. nav2_msgs/action/NavigateToPose
 * @param {object} args.message     - Goal message
 * @param {object} [args.target]    - Human-readable goal summary stored for status queries
 * @param {number} [args.timeout]   - ms, default 120 000 (only applies when blocking=true)
 * @param {boolean} [args.blocking] - If true, waits until the action completes before resolving
 * @param {object} ros              - Connected ROSLIB.Ros instance
 * @returns {Promise<{state, msg, goalId}>}
 */
export async function sendActionGoal(args, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const { device, action, actionType, message, target = {}, timeout = 120_000, blocking = false } = args;

  if (!action) throw new Error('Missing required arg: action');
  if (!actionType) throw new Error('Missing required arg: actionType');
  if (!message || typeof message !== 'object') throw new Error('Missing required arg: message (must be an object)');

  const actionServerName = device ? `/${device}/${action}` : `${action}`;

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

        resolve({
          state: 'success',
          msg: entry?.msg ?? 'Action completed',
          goalId,
        });
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
        resolve({ state: 'aborted', msg: String(error), code: error.code, goalId });
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
      resolve({ state: 'goal_sent', msg: 'Goal sent, use get_agv_action_status to track progress', goalId });
    }
  });

  return promise;
}

/**
 * Returns the current status entry for an action.
 *
 * Lookup modes:
 *  - getActionStatus({ action })         → exact key match (full name or bare action)
 *  - getActionStatus({ device })         → returns all actions registered for that device
 *  - getActionStatus({ device, action }) → exact match on /${device}/${action}
 *
 * @param {object} params
 * @param {string} [params.device]
 * @param {string} [params.action]
 */
export function getActionStatus({ device, action } = {}) {
  // device only → return all entries whose key starts with /${device}/
  if (device && !action) {
    const prefix = `/${device}/`;
    const results = {};
    for (const [key, entry] of registry) {
      if (key.startsWith(prefix)) {
        results[key] = {
          status: entry.status,
          goalId: entry.goalId,
          target: entry.target,
          startedAt: entry.startedAt,
          endedAt: entry.endedAt,
          msg: entry.msg,
        };
      }
    }
    return Object.keys(results).length
      ? results
      : { status: ActionStatus.IDLE, msg: `No actions registered for device ${device}` };
  }

  // action provided (with optional device prefix)
  const key = device ? `/${device}/${action}` : action;
  const entry = registry.get(key);
  if (!entry) return { status: ActionStatus.IDLE, msg: 'No action registered' };

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
 * Cancels the active goal for an action server (no-op if idle or already finished).
 *
 * @param {object} params
 * @param {string} [params.device]
 * @param {string} [params.action] - Required; if device provided, resolves to /${device}/${action}
 */
export function cancelAction({ device, action } = {}) {
  if (!action) return { state: 'error', msg: 'Missing required param: action' };

  const key = device ? `/${device}/${action}` : action;
  const entry = registry.get(key);
  if (!entry) return { state: 'warning', msg: `No action registered for ${key}` };
  if (entry.status !== ActionStatus.EXECUTING) {
    return { state: 'warning', msg: `Action ${key} is not executing (status: ${entry.status})` };
  }

  _cancelEntry(key, entry);
  return { state: 'success', msg: `Goal canceled for ${key}` };
}

function _cancelEntry(action, entry) {
  try {
    entry.client.cancel(entry.goalId);
  } catch (e) {
    logger.warn(`[ActionRegistry] Error canceling goal on ${action}: ${e.message}`);
  }
  entry.status = ActionStatus.CANCELING;
  entry.endedAt = new Date();
  entry.msg = 'Canceled by operator';
}
