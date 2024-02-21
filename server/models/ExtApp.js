import dotenv from 'dotenv';

const accessToken = { token: null, date: '' };

dotenv.config();

const extUrl = process.env.EXT_APP_url ?? 'http://127.0.0.1:1234';
const extUser = process.env.EXT_APP_user ?? 'user';
const extPwd = process.env.EXT_APP_pwd ?? 'password';

export class ExtApp {
  static async UpdateToken() {
    console.log(extUrl + '---' + extUser + '---' + extPwd + '--');
    let response = await fetch(`${extUrl}/token`, {
      method: 'POST',
      body: `username=${extUser}&password=${extPwd}`,
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    });
    if (response.ok) {
      let data = await response.json();
      console.log(data);
      if (data.access_token) {
        accessToken.token = data.access_token;
        accessToken.date = new Date();
      }
      return data.accessToken;
    } else {
      throw new Error(`${response.status} ${response.statusText}`);
    }
  }

  static async missionStart(missionId, mission) {
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }
    console.log(mission);
    const myMission = mission.route.map((route) => {
      let myWP = route.wp.map((wp) => ({
        latitude: wp.pos[1],
        longitude: wp.pos[0],
        altitude: wp.pos[2],
      }));
      return { uav: route.uav, wp: myWP };
    });

    let sendResponse = await fetch(`${extUrl}/drones/mission/start`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mission_id: missionId,
        routes: myMission,
      }),
    });
    if (sendResponse.ok) {
      let command = await sendResponse.json();
      console.log(command);
    } else {
      throw new Error(sendResponse.status);
    }
  }
  static async missionResult(missionId, resultCode) {
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }

    let sendResponse = await fetch(`${extUrl}/drones/mission/result`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mission_id: missionId,
        resolution_code: resultCode,
      }),
    });
    if (sendResponse.ok) {
      let command = await sendResponse.json();
      console.log(command);

      return command;
    } else {
      throw new Error(sendResponse.status);
    }
  }
  static async missionMedia(missionId, resultCode, listMedia) {
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }

    let sendResponse = await fetch(`${extUrl}/drones/mission/media`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mission_id: missionId,
        resolution_code: resultCode,
        resultados: listMedia,
      }),
    });
    if (sendResponse.ok) {
      let command = await sendResponse.json();
      console.log(command);
    } else {
      throw new Error(sendResponse.status);
    }
  }
}
