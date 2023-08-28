var mission_home = [];

export const GetMissionHome = () => {
  return mission_home;
};

export const RuteConvert = (route) => {
  let rt = [];
  console.log("version 3");
  console.log("longitud" + route.length);
  for (let n_uav = 0; n_uav < route.length; n_uav++) {
    console.log("uav - " + n_uav);
    rt[n_uav] = {};
    if (route[n_uav].hasOwnProperty("uav")) {
      rt[n_uav]["uav"] = route[n_uav].uav;
    }
    rt[n_uav]["id"] = n_uav;
    rt[n_uav]["name"] = route[n_uav].name;
    rt[n_uav]["wp"] = [];
    rt[n_uav]["attributes"] = {};

    //waipoints
    for (let wp_n = 0; wp_n < route[n_uav]["wp"].length; wp_n++) {
      rt[n_uav]["wp"][wp_n] = {};
      rt[n_uav]["wp"][wp_n]["pos"] = route[n_uav]["wp"][wp_n].pos;
      rt[n_uav]["wp"][wp_n]["yaw"] = route[n_uav]["wp"][wp_n].yaw;
      rt[n_uav]["wp"][wp_n]["gimbal"] = route[n_uav]["wp"][wp_n].gimbal;
      if (route[n_uav]["wp"][wp_n].hasOwnProperty("action")) {
        rt[n_uav]["wp"][wp_n]["action"] = route[n_uav]["wp"][wp_n].action;
      }
    }
    //    if (mission.hasOwnProperty("attributes")) {
    //      console.log("global attributes");
    //      if (mission.attributes.hasOwnProperty("mode_landing")) {
    //        console.log("global attributes mode landing");
    //        rt[n_uav]["attributes"]["mode_landing"] =
    //          mission.attributes["mode_landing"];
    //      }
    //      if (mission.attributes.hasOwnProperty("mode_yaw")) {
    //        rt[n_uav]["attributes"]["mode_yaw"] = mission.attributes["mode_yaw"];
    //      }
    //      if (mission.attributes.hasOwnProperty("mode_gimbal")) {
    //        rt[n_uav]["attributes"]["mode_gimbal"] =
    //          mission.attributes["mode_gimbal"];
    //      }
    //      if (mission.attributes.hasOwnProperty("mode_trace")) {
    //        rt[n_uav]["attributes"]["mode_trace"] =
    //          mission.attributes["mode_trace"];
    //     }
    //      if (mission.attributes.hasOwnProperty("idle_vel")) {
    //        rt[n_uav]["attributes"]["idle_vel"] = mission.attributes["idle_vel"];
    //      }
    //    }
    if (route[n_uav].hasOwnProperty("attributes")) {
      console.log("have atribute");
      if (route[n_uav]["attributes"].hasOwnProperty("mode_landing")) {
        console.log("have modelanding" + n_uav);
        rt[n_uav]["attributes"]["mode_landing"] =
          route[n_uav]["attributes"]["mode_landing"];
      }
      if (route[n_uav]["attributes"].hasOwnProperty("mode_yaw")) {
        rt[n_uav]["attributes"]["mode_yaw"] =
          route[n_uav]["attributes"]["mode_yaw"];
      }
      if (route[n_uav]["attributes"].hasOwnProperty("mode_gimbal")) {
        rt[n_uav]["attributes"]["mode_gimbal"] =
          route[n_uav]["attributes"]["mode_gimbal"];
      }
      if (route[n_uav]["attributes"].hasOwnProperty("mode_trace")) {
        rt[n_uav]["attributes"]["mode_trace"] =
          route[n_uav]["attributes"]["mode_trace"];
      }
      if (route[n_uav]["attributes"].hasOwnProperty("idle_vel")) {
        rt[n_uav]["attributes"]["idle_vel"] =
          route[n_uav]["attributes"]["idle_vel"];
      }
    } else {
      if (route[n_uav].hasOwnProperty("mode_landing")) {
        console.log("have modelanding" + n_uav);
        rt[n_uav]["attributes"]["mode_landing"] = route[n_uav]["mode_landing"];
      }
      if (route[n_uav].hasOwnProperty("mode_yaw")) {
        rt[n_uav]["attributes"]["mode_yaw"] = route[n_uav]["mode_yaw"];
      }
      if (route[n_uav].hasOwnProperty("mode_gimbal")) {
        rt[n_uav]["attributes"]["mode_gimbal"] = route[n_uav]["mode_gimbal"];
      }
      if (route[n_uav].hasOwnProperty("mode_trace")) {
        rt[n_uav]["attributes"]["mode_trace"] = route[n_uav]["mode_trace"];
      }
      if (route[n_uav].hasOwnProperty("idle_vel")) {
        rt[n_uav]["attributes"]["idle_vel"] = route[n_uav]["idle_vel"];
      }
    }
  }
  console.log("-------------------");
  console.log(rt);
  return rt;
};

export const RuteConvertlegacy = (mission) => {
  let rt = [];
  console.log("mission < 3");
  for (let n_uav = 1; n_uav <= mission["uav_n"]; n_uav++) {
    if (mission.hasOwnProperty("uav_" + n_uav)) {
      let n_uavx = n_uav - 1;
      console.log("uav - " + n_uavx);
      rt[n_uavx] = {};
      rt[n_uavx]["id"] = n_uavx;
      rt[n_uavx]["uav"] = "uav_" + n_uav;
      rt[n_uavx]["name"] = "uav_" + n_uav;
      rt[n_uavx]["wp"] = [];
      rt[n_uavx]["attributes"] = {};

      //waipoints
      for (let wp_n = 0; wp_n < mission["uav_" + n_uav]["wp_n"]; wp_n++) {
        rt[n_uavx]["wp"][wp_n] = {};
        if (mission["uav_" + n_uav]["wp_" + wp_n].length == 3) {
          rt[n_uavx]["wp"][wp_n]["yaw"] = 0;
          rt[n_uavx]["wp"][wp_n]["pos"] = mission["uav_" + n_uav]["wp_" + wp_n];
        } else {
          rt[n_uavx]["wp"][wp_n]["yaw"] =
            mission["uav_" + n_uav]["wp_" + wp_n][3];
          rt[n_uavx]["wp"][wp_n]["pos"] = mission["uav_" + n_uav][
            "wp_" + wp_n
          ].slice(0, -1);
        }
      }
      // Attributes
      if (mission.hasOwnProperty("mode_landing")) {
        rt[n_uavx]["attributes"]["mode_landing"] = mission["mode_landing"];
      }
      if (mission.hasOwnProperty("mode_yaw")) {
        rt[n_uavx]["attributes"]["mode_yaw"] = mission["mode_yaw"];
      }
      if (mission.hasOwnProperty("idle_vel")) {
        rt[n_uavx]["attributes"]["idle_vel"] = mission["idle_vel"];
      }
      if (mission["uav_" + n_uav].hasOwnProperty("attributes")) {
        if (
          mission["uav_" + n_uav]["attributes"].hasOwnProperty("mode_landing")
        ) {
          console.log("have modelanding" + n_uav);
          rt[n_uavx]["attributes"]["mode_landing"] =
            mission["uav_" + n_uav]["attributes"]["mode_landing"];
        }
        if (mission["uav_" + n_uav]["attributes"].hasOwnProperty("mode_yaw")) {
          rt[n_uavx]["attributes"]["mode_yaw"] =
            mission["uav_" + n_uav]["attributes"]["mode_yaw"];
        }
        if (mission["uav_" + n_uav]["attributes"].hasOwnProperty("idle_vel")) {
          rt[n_uavx]["attributes"]["idle_vel"] =
            mission["uav_" + n_uav]["attributes"]["idle_vel"];
        }
      } else {
        if (mission["uav_" + n_uav].hasOwnProperty("mode_landing")) {
          rt[n_uavx]["attributes"]["mode_landing"] =
            mission["uav_" + n_uav]["mode_landing"];
        }
        if (mission["uav_" + n_uav].hasOwnProperty("mode_yaw")) {
          rt[n_uavx]["attributes"]["mode_yaw"] =
            mission["uav_" + n_uav]["mode_yaw"];
        }
        if (mission["uav_" + n_uav].hasOwnProperty("idle_vel")) {
          rt[n_uavx]["attributes"]["idle_vel"] =
            mission["uav_" + n_uav]["idle_vel"];
        }
      }
    }
  }
  console.log("-----------   legacy   --------");
  console.log(rt);
  return rt;
};
