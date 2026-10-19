// Collision sensor (obstacle_info) helpers shared by the device pages and the sensor widget.

export const STOP_DISTANCE = 2; // m — below: stop
export const CAUTION_DISTANCE = 5; // m — below: caution

/** position.attributes.obstacle_info = [down, front, right, back, left, up] → named sides, or null. */
export const obstacleFromInfo = (info) =>
  info
    ? { down: info[0], front: info[1], right: info[2], back: info[3], left: info[4], up: info[5] }
    : null;

/** 'error' | 'warning' | 'primary' palette key for a distance. */
export const obstacleSeverity = (d) =>
  d < STOP_DISTANCE ? 'error' : d < CAUTION_DISTANCE ? 'warning' : 'primary';
