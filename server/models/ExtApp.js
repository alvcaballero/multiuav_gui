import { DevicesModel } from './devices.js';
import { extApp, extAppUrl, extAppUser, extAppPWD } from '../config/config.js';

const accessToken = { token: null, date: '' };

export class ExtApp {
  static async UpdateToken() {
    console.log(extAppUrl + '---' + extAppUser + '---' + extAppPWD + '--');
    let response = await fetch(`${extAppUrl}/token`, {
      method: 'POST',
      body: `username=${extAppUser}&password=${extAppPWD}`,
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    });
    if (response.ok) {
      console.log('response of Tocken ------------');
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
    console.log('Mission Start');
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }
    const myMission = mission.route.map((route) => {
      let myWP = route.wp.map((wp) => ({
        latitude: wp.pos[1],
        longitude: wp.pos[0],
        altitude: wp.pos[2],
      }));
      let myDevice = DevicesModel.getByName(route.uav);
      return { deviceId: myDevice.id, wp: myWP };
    });

    console.log('mission_id: ' + missionId);
    console.log('routes');

    console.log(myMission);

    let sendResponse = await fetch(`${extAppUrl}/drones/mission/start`, {
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
      console.log('response OK Send start to external applications');
      //let command = await sendResponse.json();
      //console.log(command);
    } else {
      throw new Error(sendResponse.status);
    }
  }
  static async missionResult(missionId, resultCode) {
    console.log('Mission Result');
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }
    let request = {
      mission_id: missionId,
      resolution_code: resultCode,
    };
    console.log(request);
    let sendResponse = await fetch(`${extAppUrl}/drones/mission/result`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });
    if (sendResponse.ok) {
      console.log('Response ok to send result of mission');
      //let command = await sendResponse.json();
      //console.log(command);
    } else {
      throw new Error(sendResponse.status);
    }
  }
  static async missionMedia(missionId, results) {
    console.log('Mission Media');
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }
    let request = {
      mission_id: missionId,
      files: results.files,
      result: results.data,
    };
    console.log(request);

    let sendResponse = await fetch(`${extAppUrl}/drones/mission/media`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });
    if (sendResponse.ok) {
      console.log('Response ok to send media of mission -----');
      //let command = await sendResponse.json();
      //console.log(command);
    } else {
      throw new Error(sendResponse.status);
    }
  }
}
