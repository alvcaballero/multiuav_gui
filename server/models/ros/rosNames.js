// Every ROS name of a device, built in ONE place. Pure: the facade (ros.js) is the only caller.

// Device names are stored WITHOUT a leading slash and every config `name` starts with one, so the
// result is always absolute (`/agv_1/odom`). Topics, services and actions alike.
export function buildDeviceName(name, entry) {
  return `/${name}${entry['name']}`;
}

// A device whose companion runs the slow relay (`attributes.slow`, ROS only) gets its telemetry
// republished at a lower rate under `/<device>/slow<topic>`: only the slow copies cross the link, as
// rosbridge runs next to the GCS (in ROS 1 a subscriber receives every message of its topic). Only
// the SUBSCRIBERS move; services, actions and publishers keep their names.
export const SLOW_RELAY_NAMESPACE = '/slow';

export function buildTelemetryName(name, entry, attributes) {
  return attributes?.slow ? `/${name}${SLOW_RELAY_NAMESPACE}${entry['name']}` : buildDeviceName(name, entry);
}
