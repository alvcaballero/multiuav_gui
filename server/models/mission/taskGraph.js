export const TASK_GRAPH_VERSION = '4';
export const LEGACY_ROUTE_ACTION = 'ROUTE';

export class TaskGraphError extends Error {
  constructor(errors) {
    super(`Invalid task graph: ${errors.join('; ')}`);
    this.name = 'TaskGraphError';
    this.errors = errors;
    this.status = 400;
  }
}

// task_id comes from the array index, not route.id: planners (LLM) have been seen
// emitting duplicate or missing route ids, and the index is unique by construction.
export function routeToTask(route, index) {
  const { uav, attributes, wp, id: _legacyId, ...rest } = route;
  return {
    ...rest,
    task_id: `T${index + 1}`,
    device: uav,
    action: LEGACY_ROUTE_ACTION,
    depends_on: [],
    params: attributes ?? {},
    wp: wp ?? [],
  };
}

export function normalizeMission(missionData) {
  if (missionData == null || typeof missionData !== 'object') {
    throw new TaskGraphError(['mission must be an object']);
  }
  const { route, tasks, version: _version, ...rest } = missionData;
  const hasTasks = Array.isArray(tasks);
  const hasRoutes = Array.isArray(route);

  if (hasTasks && hasRoutes) {
    throw new TaskGraphError(['mission has both tasks[] and route[]; send only one']);
  }
  if (!hasTasks && !hasRoutes) {
    throw new TaskGraphError(['mission must have tasks[] or route[]']);
  }

  const normalizedTasks = hasTasks
    ? tasks.map((t) => ({ ...t, depends_on: t?.depends_on ?? [] }))
    : route.map(routeToTask);

  const errors = validateTaskGraph(normalizedTasks);
  if (errors.length > 0) throw new TaskGraphError(errors);

  return { ...rest, version: TASK_GRAPH_VERSION, tasks: normalizedTasks };
}

/**
 * The one-task mission a single device gets loaded with. depends_on is dropped:
 * the tasks it points to are not part of this payload, and ordering is the
 * scheduler's job, not the device's.
 * @param {object} mission - a normalized mission (see normalizeMission)
 */
export function missionForTask(mission, taskId) {
  const task = mission.tasks.find((t) => t.task_id === taskId);
  if (!task) return null;
  return { ...mission, tasks: [{ ...task, depends_on: [] }] };
}

/**
 * Returns a list of human-readable errors (empty when valid). Graph-level checks
 * (cycles, same-device ordering) only run once every task is structurally sound.
 */
export function validateTaskGraph(tasks) {
  if (!Array.isArray(tasks) || tasks.length === 0) {
    return ['tasks must be a non-empty array'];
  }

  const errors = [];
  const ids = new Set();
  for (const [i, task] of tasks.entries()) {
    const label = typeof task?.task_id === 'string' && task.task_id ? task.task_id : `tasks[${i}]`;
    if (typeof task?.task_id !== 'string' || task.task_id.trim() === '') {
      errors.push(`${label}: task_id must be a non-empty string`);
    } else if (ids.has(task.task_id)) {
      errors.push(`${label}: duplicate task_id`);
    } else {
      ids.add(task.task_id);
    }
    if (typeof task?.device !== 'string' || task.device.trim() === '') {
      errors.push(`${label}: device must be a non-empty string`);
    }
    if (!Array.isArray(task?.wp) || task.wp.length === 0) {
      errors.push(`${label}: wp must be a non-empty array`);
    }
    if (!Array.isArray(task?.depends_on)) {
      errors.push(`${label}: depends_on must be an array`);
    }
    // Without this, a planner still speaking the route vocabulary would silently
    // lose its flight params (max_vel, mode_yaw...) and fly on defaults.
    if (task?.attributes !== undefined && task?.params === undefined) {
      errors.push(`${label}: uses 'attributes'; tasks use 'params'`);
    }
  }
  if (errors.length > 0) return errors;

  for (const task of tasks) {
    for (const dep of task.depends_on) {
      if (dep === task.task_id) errors.push(`${task.task_id}: depends on itself`);
      else if (!ids.has(dep)) errors.push(`${task.task_id}: depends on unknown task '${dep}'`);
    }
  }
  if (errors.length > 0) return errors;

  const order = topologicalOrder(tasks);
  if (order.length < tasks.length) {
    const sorted = new Set(order);
    const inCycle = tasks.filter((t) => !sorted.has(t.task_id)).map((t) => t.task_id);
    return [`dependency cycle among tasks: ${inCycle.join(', ')}`];
  }

  // A device runs one task at a time, so its tasks must form a chain in the graph;
  // two unordered tasks on the same device would be dispatched concurrently.
  const ancestors = ancestorsMap(tasks, order);
  const byDevice = groupByDevice(tasks);
  for (const [device, deviceTasks] of byDevice) {
    for (let i = 0; i < deviceTasks.length; i++) {
      for (let j = i + 1; j < deviceTasks.length; j++) {
        const a = deviceTasks[i].task_id;
        const b = deviceTasks[j].task_id;
        if (!ancestors.get(a).has(b) && !ancestors.get(b).has(a)) {
          errors.push(`device '${device}': tasks ${a} and ${b} are not ordered by depends_on`);
        }
      }
    }
  }
  return errors;
}

// Kahn's algorithm. Returns task_ids in dependency order; shorter than `tasks`
// when there is a cycle.
export function topologicalOrder(tasks) {
  const inDegree = new Map(tasks.map((t) => [t.task_id, t.depends_on.length]));
  const children = dependentsMap(tasks);
  const queue = tasks.filter((t) => t.depends_on.length === 0).map((t) => t.task_id);
  const order = [];
  while (queue.length > 0) {
    const id = queue.shift();
    order.push(id);
    for (const child of children.get(id)) {
      inDegree.set(child, inDegree.get(child) - 1);
      if (inDegree.get(child) === 0) queue.push(child);
    }
  }
  return order;
}

export function descendants(tasks, taskId) {
  const children = dependentsMap(tasks);
  const seen = new Set();
  const stack = [...(children.get(taskId) ?? [])];
  while (stack.length > 0) {
    const id = stack.pop();
    if (seen.has(id)) continue;
    seen.add(id);
    stack.push(...children.get(id));
  }
  return [...seen];
}

/**
 * Tasks whose own status is still pending and whose every dependency is done.
 * Status-agnostic on purpose: the caller decides what counts as pending/done.
 * @param {object[]} tasks
 * @param {{ pending: Set<string>, done: Set<string> }} state - sets of task_ids
 */
export function readyTasks(tasks, { pending, done }) {
  return tasks.filter((t) => pending.has(t.task_id) && t.depends_on.every((dep) => done.has(dep)));
}

// Relies on the same-device ordering invariant enforced by validateTaskGraph:
// any later task on this device is necessarily a descendant.
export function isLastTaskOfDevice(tasks, taskId) {
  const task = tasks.find((t) => t.task_id === taskId);
  if (!task) return false;
  const later = new Set(descendants(tasks, taskId));
  return !tasks.some((t) => t.device === task.device && later.has(t.task_id));
}

function dependentsMap(tasks) {
  const children = new Map(tasks.map((t) => [t.task_id, []]));
  for (const task of tasks) {
    for (const dep of task.depends_on) children.get(dep)?.push(task.task_id);
  }
  return children;
}

function ancestorsMap(tasks, order) {
  const byId = new Map(tasks.map((t) => [t.task_id, t]));
  const ancestors = new Map();
  for (const id of order) {
    const set = new Set();
    for (const dep of byId.get(id).depends_on) {
      set.add(dep);
      for (const a of ancestors.get(dep)) set.add(a);
    }
    ancestors.set(id, set);
  }
  return ancestors;
}

function groupByDevice(tasks) {
  const byDevice = new Map();
  for (const task of tasks) {
    if (!byDevice.has(task.device)) byDevice.set(task.device, []);
    byDevice.get(task.device).push(task);
  }
  return byDevice;
}
