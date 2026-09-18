import palette from '../../shared/palette';

const createFeature = (myroute, point) => {
  let myYaw = null;
  let mySpeed = 0;
  if (
    myroute[point.routeid].wp[point.id].hasOwnProperty('action') &&
    myroute[point.routeid].wp[point.id].action?.hasOwnProperty('yaw')
  ) {
    myYaw = myroute[point.routeid].wp[point.id].action.yaw;
  } else if (myroute[point.routeid].wp[point.id].hasOwnProperty('yaw')) {
    myYaw = myroute[point.routeid].wp[point.id].yaw;
  }
  if (myroute[point.routeid].wp[point.id].hasOwnProperty('speed')) {
    mySpeed = myroute[point.routeid].wp[point.id].speed;
  } else if (
    myroute[point.routeid].attributes &&
    myroute[point.routeid].attributes.hasOwnProperty('idle_vel')
  ) {
    mySpeed = myroute[point.routeid].attributes.idle_vel;
  }

  // null/undefined stays null (waypoint has no heading -> drawn as a plain point);
  // any real number, including 0 (north), gets an oriented arrow icon.
  myYaw = myYaw != null && Number.isFinite(Number(myYaw)) ? Number(myYaw) : null;
  mySpeed = Number(mySpeed) ? Number(mySpeed) : 0;
  const myCategory = myYaw == null ? 'background' : 'backgroundDirection';
  return {
    id: point.id,
    route_id: point.routeid,
    name: myroute[point.routeid].name,
    uav: myroute[point.routeid].uav,
    latitude: myroute[point.routeid].wp[point.id].pos[0],
    longitude: myroute[point.routeid].wp[point.id].pos[1],
    altitude: myroute[point.routeid].wp[point.id].pos[2],
    yaw: myroute[point.routeid].wp[point.id].yaw,
    speed: mySpeed,
    gimbal: myroute[point.routeid].wp[point.id].gimbal,
    actions: myroute[point.routeid].wp[point.id].action,
    attributes: myroute[point.routeid].attributes,
    category: myCategory,
    rotation: myYaw ?? 0,
    color: point.routeid, //myroute[point.routeid]['id'],
  };
};

/**
 * Escapes a value for interpolation into popup markup.
 *
 * Popup content is NOT trusted: waypoint names, action keys and mission attributes
 * travel with the mission, and missions in this system are produced by the LLM
 * planner from tool output and operator text. The result is handed to
 * `maplibregl.Popup().setHTML()`, whose sanitizer has a known bypass in the version
 * pinned here — so escaping happens at the source instead of relying on it.
 *
 * @param {*} value
 * @returns {string} HTML-safe text
 */
const escapeHtml = (value) =>
  String(value ?? '').replace(
    /[&<>"']/g,
    (char) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[char],
  );

/** Renders `key: value unit` rows, escaping BOTH sides — the keys are data too. */
const renderRows = (entries) =>
  Object.keys(entries || {})
    .map((key) => {
      const unit = key === 'idle_vel' || key === 'max_vel' ? 'm/s' : '';
      return `<div style="display:inline"><span>${escapeHtml(key)}: </span><span>${escapeHtml(
        entries[key],
      )} ${unit} </span></div>`;
    })
    .join('');

const textPopUp = ({ properties, attributes, actions }) => {
  // Coordinates land inside an href, where escaping alone would not stop an
  // attribute break-out, so they are forced to finite numbers or dropped.
  const lat = Number(properties.latitude);
  const lon = Number(properties.longitude);
  const mapsQuery = Number.isFinite(lat) && Number.isFinite(lon) ? `${lat},${lon}` : '';

  let html = `<div style="color: #FF7A59;text-align: center" ><b>UAV: ${escapeHtml(properties.uav)}</b>
    <span><a href="https://www.google.com/maps?q=${mapsQuery}" target="_blank">
    Point_${escapeHtml(properties.id)}</a></span></div>
        <div><span>Route: ${escapeHtml(properties.name)}</span></div>
        <div style="display:inline"><span> Height: </span><span>${properties.altitude.toFixed(1)}m </span></div>`;
  html = properties.hasOwnProperty('speed')
    ? `${html} <div style="display:inline"><span>Speed: </span><span>${escapeHtml(properties.speed)} m/s </span></div>`
    : html;
  html = properties.hasOwnProperty('yaw')
    ? `${html} <div style="display:inline"><span>Yaw: </span><span>${escapeHtml(properties.yaw)}° </span></div>`
    : html;
  html = properties.hasOwnProperty('gimbal')
    ? `${html} <div style="display:inline"><span>Gimbal: </span><span>${escapeHtml(properties.gimbal)}° </span></div>`
    : html;

  // `attributes` is null for a route that carries none (see MapMissions.WaypointDetail),
  // which used to throw inside Object.keys and take the whole popup down.
  const htmlAction = `<div><b> Waypoint actions: </b></div>${actions ? renderRows(actions) : ''}`;
  const htmlAttributes = `<div><b> Attributes_mission: </b></div>${renderRows(attributes)}`;

  return html + htmlAction + htmlAttributes;
};

function routesToFeature(item) {
  const waypointPos = item.wp.map((it) => [it.pos[1], it.pos[0]]);

  return {
    id: item.id,
    type: 'Feature',
    geometry: {
      type: 'LineString',
      coordinates: waypointPos,
    },
    properties: {
      name: item.uav, //name,
      color: palette.colors_devices[+item.id % Object.keys(palette.colors_devices).length],
    },
  };
}

function routeTowaypoints(myroute) {
  const waypoint = [];
  myroute.forEach((rt, indexRt) => {
    rt.wp.forEach((wp, indexWp) => {
      waypoint.push({
        longitude: wp.pos[1],
        latitude: wp.pos[0],
        id: indexWp,
        routeid: indexRt,
      });
    });
  });
  return waypoint;
}

function cleanRoute(myroute) {
  const myRoutes = structuredClone(myroute);
  for (let i = 0; i < myroute.length; i += 1) {
    if (!myroute[i].hasOwnProperty('id')) {
      myRoutes[i].id = i;
    }
  }
  return myRoutes;
}

export { createFeature, textPopUp, routesToFeature, routeTowaypoints, cleanRoute };
