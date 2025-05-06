
import palette from '../../common/palette';

const createFeature = (myroute, point) => {
    let myYaw = 0;
    let mySpeed = 0;
    if (
      myroute[point.routeid].wp[point.id].hasOwnProperty('action') &&
      myroute[point.routeid].wp[point.id].action.hasOwnProperty('yaw')
    ) {
      myYaw = myroute[point.routeid].wp[point.id].action.yaw;
    } else if (myroute[point.routeid].wp[point.id].hasOwnProperty('yaw')) {
      myYaw = myroute[point.routeid].wp[point.id].yaw;
    }
    if (myroute[point.routeid].wp[point.id].hasOwnProperty('speed')) {
      mySpeed = myroute[point.routeid].wp[point.id].speed;
    } else if (myroute[point.routeid].attributes && myroute[point.routeid].attributes.hasOwnProperty('idle_vel')) {
      mySpeed = myroute[point.routeid].attributes.idle_vel;
    }

    myYaw = Number(myYaw) ? Number(myYaw) : 0;
    mySpeed = Number(mySpeed) ? Number(mySpeed) : 0;
    const myCategory = myYaw === 0 ? 'background' : 'backgroundDirection';
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
      rotation: myYaw,
      color: point.routeid,//myroute[point.routeid]['id'],
    };
  };

  const textPopUp = ({properties,attributes,actions}) => {
    let html = `<div style="color: #FF7A59;text-align: center" ><b>UAV: ${properties.uav}</b>
    <span><a href="https://www.google.com/maps?q=${properties.latitude},${properties.longitude}" target="_blank">
    Point_${properties.id}</a></span></div>
        <div><span>Route: ${properties.name}</span></div>
        <div style="display:inline"><span> Height: </span><span>${properties.altitude.toFixed(1)}m </span></div>`;
  html = properties.hasOwnProperty('speed')
    ? `${html} <div style="display:inline"><span>Speed: </span><span>${properties.speed} m/s </span></div>`
    : html;
  html = properties.hasOwnProperty('yaw')
    ? `${html} <div style="display:inline"><span>Yaw: </span><span>${properties.yaw}° </span></div>`
    : html;
  html = properties.hasOwnProperty('gimbal')
    ? `${html} <div style="display:inline"><span>Gimbal: </span><span>${properties.gimbal}° </span></div>`
    : html;
  let htmlAction = '<div><b> Waypoint actions: </b></div>';
  if (actions) {
    htmlAction += Object.keys(actions)
      .map((key) => {
        const unit = key === 'idle_vel' ? 'm/s' : '';
        return `<div style="display:inline"><span>${key}: </span><span>${actions[key]} ${unit} </span></div>`;
      })
      .join('');
  }
  let htmlAttributes = '<div><b> Attributes_mission: </b></div>';
  htmlAttributes += Object.keys(attributes)
    .map((key) => {
      const unit = key === 'idle_vel' || key === 'max_vel' ? 'm/s' : '';
      return `<div style="display:inline"><span>${key}: </span><span>${attribute[key]} ${unit} </span></div>`;
    })
    .join('');
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
    const myRoutes = JSON.parse(JSON.stringify(myroute));
    for (let i = 0; i < myroute.length; i += 1) {
      if (!myroute[i].hasOwnProperty('id')) {
        myRoutes[i].id = i;
      }
    }
    return myRoutes;
  }

export { createFeature, textPopUp, routesToFeature, routeTowaypoints, cleanRoute };