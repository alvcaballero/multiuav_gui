// Lista de atributos posibles
const ROUTE_CONVERT_ATTRS = [
  'mode_landing',
  'mode_yaw',
  'mode_gimbal',
  'mode_trace',
  'idle_vel',
  'max_vel',
];

export const RuteConvert = (data) => {
  if (data.version == '3') {
    return RuteConvertv3(data.route);
  }
  return RuteConvertlegacy(data);
};

export const RuteConvertv3 = (route) => {
  const rt = [];
  let latlongError = false;

  for (let uavN = 0; uavN < route.length; uavN++) {
    const src = route[uavN];
    const dst = {
      id: uavN,
      name: src.name,
      wp: [],
      attributes: {},
    };
    if ('uav' in src) dst.uav = src.uav;

    // Waypoints
    if (Array.isArray(src.wp)) {
      for (let wpN = 0; wpN < src.wp.length; wpN++) {
        const wpSrc = src.wp[wpN];
        const wpDst = {
          pos: wpSrc.pos,
          yaw: wpSrc.yaw,
          gimbal: wpSrc.gimbal,
        };
        // Validación de lat/lon
        if (Math.abs(Number(wpSrc.pos[0])) > 90 || Math.abs(Number(wpSrc.pos[1])) > 90) {
          console.log(
            `Error en latitud o longitud en UAV ${uavN + 1} WP ${wpN + 1} pos ${wpSrc.pos}`,
          );
          latlongError = true;
        }
        if ('speed' in wpSrc) wpDst.speed = wpSrc.speed;
        if ('action' in wpSrc) wpDst.action = wpSrc.action;
        dst.wp.push(wpDst);
      }
    }

    // Atributos
    const attrSrc = src.attributes || src;
    ROUTE_CONVERT_ATTRS.forEach((key) => {
      if (key in attrSrc) {
        dst.attributes[key] = attrSrc[key];
        if (key === 'mode_landing' && src.attributes) {
          console.log('have modelanding' + uavN);
        }
      }
    });

    rt.push(dst);
  }

  if (latlongError) {
    console.error('Error en coordenadas latitud y longitud valores entre -90 y 90');
    return [];
  }
  return rt;
};

const LEGACY_CONVERT_ATTRS = ['mode_landing', 'mode_yaw', 'idle_vel', 'max_vel'];

const RuteConvertlegacy = (mission) => {
  const rt = [];

  for (let uavN = 1; uavN <= mission['uav_n']; uavN++) {
    const uavKey = 'uav_' + uavN;
    if (!mission.hasOwnProperty(uavKey)) continue;

    const uavNx = uavN - 1;
    const uavData = mission[uavKey];
    const wpCount = uavData['wp_n'] || 0;

    const uavObj = {
      id: uavNx,
      uav: uavKey,
      name: uavKey,
      wp: [],
      attributes: {},
    };

    // Waypoints
    for (let wpN = 0; wpN < wpCount; wpN++) {
      const wp = uavData['wp_' + wpN];
      if (!wp) continue;
      const pos = wp.length === 3 ? wp : wp.slice(0, -1);
      const yaw = wp.length === 3 ? 0 : wp[3];
      uavObj.wp.push({ pos, yaw });
    }

    // Global attributes
    LEGACY_CONVERT_ATTRS.forEach((attr) => {
      if (mission.hasOwnProperty(attr) && attr !== 'max_vel') {
        uavObj.attributes[attr] = mission[attr];
      }
    });

    // UAV-specific attributes
    const attrSrc = uavData.attributes || uavData;
    LEGACY_CONVERT_ATTRS.forEach((attr) => {
      if (attr in attrSrc) {
        uavObj.attributes[attr] = attrSrc[attr];
        if (attr === 'mode_landing' && uavData.attributes) {
          console.log('have modelanding' + uavN);
        }
      }
    });

    rt[uavNx] = uavObj;
  }

  console.log('-----------   legacy   --------');
  console.log(rt);
  return rt;
};
