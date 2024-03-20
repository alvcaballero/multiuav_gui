var mission_home = [];

export const GetMissionHome = () => {
  return mission_home;
};

export const RuteConvert = (route) => {
  let rt = [];
  for (let uavN = 0; uavN < route.length; uavN++) {
    rt[uavN] = {};
    if (route[uavN].hasOwnProperty('uav')) {
      rt[uavN]['uav'] = route[uavN].uav;
    }
    rt[uavN]['id'] = uavN;
    rt[uavN]['name'] = route[uavN].name;
    rt[uavN]['wp'] = [];
    rt[uavN]['attributes'] = {};

    //waipoints
    if (Array.isArray(route[uavN]['wp'])) {
      for (let wpN = 0; wpN < route[uavN]['wp'].length; wpN++) {
        rt[uavN]['wp'][wpN] = {};
        rt[uavN]['wp'][wpN]['pos'] = route[uavN]['wp'][wpN].pos;
        rt[uavN]['wp'][wpN]['yaw'] = route[uavN]['wp'][wpN].yaw;
        rt[uavN]['wp'][wpN]['gimbal'] = route[uavN]['wp'][wpN].gimbal;
        if (route[uavN]['wp'][wpN].hasOwnProperty('speed')) {
          rt[uavN]['wp'][wpN]['speed'] = route[uavN]['wp'][wpN].speed;
        }
        if (route[uavN]['wp'][wpN].hasOwnProperty('action')) {
          rt[uavN]['wp'][wpN]['action'] = route[uavN]['wp'][wpN].action;
        }
      }
    }
    if (route[uavN].hasOwnProperty('attributes')) {
      if (route[uavN]['attributes'].hasOwnProperty('mode_landing')) {
        console.log('have modelanding' + uavN);
        rt[uavN]['attributes']['mode_landing'] = route[uavN]['attributes']['mode_landing'];
      }
      if (route[uavN]['attributes'].hasOwnProperty('mode_yaw')) {
        rt[uavN]['attributes']['mode_yaw'] = route[uavN]['attributes']['mode_yaw'];
      }
      if (route[uavN]['attributes'].hasOwnProperty('mode_gimbal')) {
        rt[uavN]['attributes']['mode_gimbal'] = route[uavN]['attributes']['mode_gimbal'];
      }
      if (route[uavN]['attributes'].hasOwnProperty('mode_trace')) {
        rt[uavN]['attributes']['mode_trace'] = route[uavN]['attributes']['mode_trace'];
      }
      if (route[uavN]['attributes'].hasOwnProperty('idle_vel')) {
        rt[uavN]['attributes']['idle_vel'] = route[uavN]['attributes']['idle_vel'];
      }
      if (route[uavN]['attributes'].hasOwnProperty('max_vel')) {
        rt[uavN]['attributes']['max_vel'] = route[uavN]['attributes']['max_vel'];
      }
    } else {
      if (route[uavN].hasOwnProperty('mode_landing')) {
        rt[uavN]['attributes']['mode_landing'] = route[uavN]['mode_landing'];
      }
      if (route[uavN].hasOwnProperty('mode_yaw')) {
        rt[uavN]['attributes']['mode_yaw'] = route[uavN]['mode_yaw'];
      }
      if (route[uavN].hasOwnProperty('mode_gimbal')) {
        rt[uavN]['attributes']['mode_gimbal'] = route[uavN]['mode_gimbal'];
      }
      if (route[uavN].hasOwnProperty('mode_trace')) {
        rt[uavN]['attributes']['mode_trace'] = route[uavN]['mode_trace'];
      }
      if (route[uavN].hasOwnProperty('idle_vel')) {
        rt[uavN]['attributes']['idle_vel'] = route[uavN]['idle_vel'];
      }
      if (route[uavN].hasOwnProperty('max_vel')) {
        rt[uavN]['attributes']['max_vel'] = route[uavN]['max_vel'];
      }
    }
  }
  return rt;
};

export const RuteConvertlegacy = (mission) => {
  let rt = [];
  console.log('mission < 3');
  for (let uavN = 1; uavN <= mission['uav_n']; uavN++) {
    if (mission.hasOwnProperty('uav_' + uavN)) {
      let uavNx = uavN - 1;
      console.log('uav - ' + uavNx);
      rt[uavNx] = {};
      rt[uavNx]['id'] = uavNx;
      rt[uavNx]['uav'] = 'uav_' + uavN;
      rt[uavNx]['name'] = 'uav_' + uavN;
      rt[uavNx]['wp'] = [];
      rt[uavNx]['attributes'] = {};

      //waipoints
      for (let wpN = 0; wpN < mission['uav_' + uavN]['wp_n']; wpN++) {
        rt[uavNx]['wp'][wpN] = {};
        if (mission['uav_' + uavN]['wp_' + wpN].length == 3) {
          rt[uavNx]['wp'][wpN]['yaw'] = 0;
          rt[uavNx]['wp'][wpN]['pos'] = mission['uav_' + uavN]['wp_' + wpN];
        } else {
          rt[uavNx]['wp'][wpN]['yaw'] = mission['uav_' + uavN]['wp_' + wpN][3];
          rt[uavNx]['wp'][wpN]['pos'] = mission['uav_' + uavN]['wp_' + wpN].slice(0, -1);
        }
      }
      // Attributes
      if (mission.hasOwnProperty('mode_landing')) {
        rt[uavNx]['attributes']['mode_landing'] = mission['mode_landing'];
      }
      if (mission.hasOwnProperty('mode_yaw')) {
        rt[uavNx]['attributes']['mode_yaw'] = mission['mode_yaw'];
      }
      if (mission.hasOwnProperty('idle_vel')) {
        rt[uavNx]['attributes']['idle_vel'] = mission['idle_vel'];
      }
      if (mission['uav_' + uavN].hasOwnProperty('attributes')) {
        if (mission['uav_' + uavN]['attributes'].hasOwnProperty('mode_landing')) {
          console.log('have modelanding' + uavN);
          rt[uavNx]['attributes']['mode_landing'] =
            mission['uav_' + uavN]['attributes']['mode_landing'];
        }
        if (mission['uav_' + uavN]['attributes'].hasOwnProperty('mode_yaw')) {
          rt[uavNx]['attributes']['mode_yaw'] = mission['uav_' + uavN]['attributes']['mode_yaw'];
        }
        if (mission['uav_' + uavN]['attributes'].hasOwnProperty('idle_vel')) {
          rt[uavNx]['attributes']['idle_vel'] = mission['uav_' + uavN]['attributes']['idle_vel'];
        }
        if (mission['uav_' + uavN]['attributes'].hasOwnProperty('max_vel')) {
          rt[uavN]['attributes']['max_vel'] = mission['uav_' + uavN]['attributes']['max_vel'];
        }
      } else {
        if (mission['uav_' + uavN].hasOwnProperty('mode_landing')) {
          rt[uavNx]['attributes']['mode_landing'] = mission['uav_' + uavN]['mode_landing'];
        }
        if (mission['uav_' + uavN].hasOwnProperty('mode_yaw')) {
          rt[uavNx]['attributes']['mode_yaw'] = mission['uav_' + uavN]['mode_yaw'];
        }
        if (mission['uav_' + uavN].hasOwnProperty('idle_vel')) {
          rt[uavNx]['attributes']['idle_vel'] = mission['uav_' + uavN]['idle_vel'];
        }
      }
    }
  }
  console.log('-----------   legacy   --------');
  console.log(rt);
  return rt;
};
