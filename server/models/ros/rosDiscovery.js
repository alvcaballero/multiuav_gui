import { getTopics, getPublishers } from './rosInspect.js';
import { logger } from '../../common/logger.js';

// Waits until somebody publishes a set of topics, without subscribing to them. A subscriber is
// itself a node of the ROS graph (it shows in `ros2 topic list`), so subscribing to the telemetry
// of every registered device fills the graph with topics nobody publishes. The facade (ros.js)
// resolves the names and what to do once they are alive; this primitive only watches.
//
// It leans on rosapi, which is fragile (it dies on a bad request, and then a call never answers):
// every call has a timeout, a tick never overlaps the previous one, and a failure only skips the
// tick (the watched keys stay pending and the next tick tries again).

const CALL_TIMEOUT_MS = 3000;

// watched[key] = { topics: [name], onFound }
const watched = new Map();
let timer = null;
let ticking = false;
let rosapiFailing = false;

function withTimeout(promise, label) {
  let id;
  const timeout = new Promise((_, reject) => {
    id = setTimeout(() => reject(new Error(`${label} timed out after ${CALL_TIMEOUT_MS}ms`)), CALL_TIMEOUT_MS);
  });
  return Promise.race([promise, timeout]).finally(() => clearTimeout(id));
}

async function hasPublisher(topic, ros) {
  const publishers = await withTimeout(getPublishers(topic, ros), `rosapi publishers ${topic}`);
  return Array.isArray(publishers) && publishers.length > 0;
}

async function tick(getRos) {
  if (ticking) return;
  const ros = getRos();
  if (!ros || !ros.isConnected) return;
  ticking = true;
  try {
    // One call for the whole graph first: only the names that exist are worth a publishers call.
    // The list also holds topics that have subscribers and no publisher, hence the second check.
    const graph = await withTimeout(getTopics(ros), 'rosapi topics');
    const existing = new Set(graph.topics);
    for (const [key, entry] of [...watched]) {
      let alive = null;
      for (const topic of entry.topics) {
        if (!existing.has(topic)) continue;
        if (!watched.has(key)) break; // unwatched while a call was in flight
        if (await hasPublisher(topic, ros)) {
          alive = topic;
          break;
        }
      }
      if (alive && watched.delete(key)) {
        try {
          await entry.onFound(alive);
        } catch (error) {
          logger.error(`ROS discovery: activating ${key} failed: ${error.message}`);
        }
      }
    }
    if (rosapiFailing) {
      rosapiFailing = false;
      logger.info('ROS discovery: rosapi answers again');
    }
  } catch (error) {
    if (!rosapiFailing) {
      rosapiFailing = true;
      logger.warn(`ROS discovery: rosapi failed (${error.message}); devices stay pending and it retries. Set ROS_LAZY_SUBSCRIBE=false to subscribe without it`);
    }
  } finally {
    ticking = false;
    if (watched.size === 0) stopTimer();
  }
}

function stopTimer() {
  clearInterval(timer);
  timer = null;
}

// Watch `topics` under `key`: the first one that has a publisher removes the key and calls
// onFound(topic). A watch is one-shot — after onFound the key is the caller's again.
export function watchPublishers({ key, topics }, onFound, getRos, intervalMs) {
  watched.set(String(key), { topics, onFound });
  if (!timer) {
    timer = setInterval(() => tick(getRos), intervalMs);
    timer.unref?.();
    // Do not wait a whole interval for the first look: the device may be up already.
    tick(getRos);
  }
}

export function unwatch(key) {
  watched.delete(String(key));
  if (watched.size === 0) stopTimer();
}

export function unwatchAll() {
  watched.clear();
  stopTimer();
}

export function pendingKeys() {
  return [...watched.keys()];
}
