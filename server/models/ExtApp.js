import { devicesController } from '../controllers/devices.js';
import { extApp, extAppUrl, extAppUser, extAppPWD } from '../config/config.js';
import logger from '../common/logger.js';

const accessToken = { token: null, date: '' };

const AppFetch = async (url, attributes) => {
  if (!extApp) {
    logger.debug('no third party application configured');
    const obj = { access_token: 'world' };
    const myBlob = new Blob([JSON.stringify(obj, null, 2)], {
      type: 'application/json',
    });
    const myOptions = { status: 200, statusText: 'SuperSmashingGreat!' };
    return new Response(myBlob, myOptions);
  }
  return await fetch(url, attributes);
};

export class ExtApp {
  static async UpdateToken() {
    logger.debug(`UpdateToken url=${extAppUrl} user=${extAppUser}`);
    let response = await AppFetch(`${extAppUrl}/token`, {
      method: 'POST',
      body: `username=${extAppUser}&password=${extAppPWD}`,
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    });
    if (response.ok) {
      logger.info('ExtApp token updated successfully');
      let data = await response.json();
      logger.debug(`UpdateToken response data: ${JSON.stringify(data)}`);
      if (data.access_token) {
        accessToken.token = data.access_token;
        accessToken.date = new Date();
      }
      return data.accessToken;
    } else {
      logger.error('Error getting ExtApp token');
      throw new Error(`${response.status} ${response.statusText}`);
    }
  }

  // `externalId` is the task id the external system assigned — it is what ExtApp
  // must be addressed by, NOT our internal mission PK. Callers (missionModel)
  // translate PK → externalId before invoking these methods.
  static async missionStart(externalId, mission) {
    logger.info(`missionStart externalId=${externalId}`);
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }
    let myMission = [];
    for (let route of mission.route) {
      let myWP = route.wp.map((wp) => ({
        latitude: wp.pos[1],
        longitude: wp.pos[0],
        altitude: Number(wp.pos[2]).toFixed(),
      }));

      let myDevice = await devicesController.getByName(route.uav);
      myMission.push({ deviceId: myDevice.id, wp: myWP });
    }

    logger.debug(`missionStart sending mission_id=${externalId} routes: ${JSON.stringify(myMission)}`);

    let sendResponse = await AppFetch(`${extAppUrl}/drones/mission/start`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mission_id: Number(externalId),
        routes: myMission,
      }),
    });
    if (sendResponse.ok) {
      logger.info('missionStart sent to external application successfully');
    } else {
      logger.error('error sending mission start to external application');
    }
  }
  static async missionResult(externalId, resultCode) {
    logger.info(`missionResult externalId=${externalId} resultCode=${resultCode}`);
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }
    let request = {
      mission_id: externalId,
      resolution_code: resultCode,
    };
    logger.debug(`missionResult request: ${JSON.stringify(request)}`);
    let sendResponse = await AppFetch(`${extAppUrl}/drones/mission/result`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });
    if (sendResponse.ok) {
      logger.info('missionResult sent to external application successfully');
    } else {
      logger.error('error sending mission result to external application');
    }
  }
  static async missionMedia(externalId, results) {
    logger.info(`missionMedia externalId=${externalId}`);
    if (accessToken.token) {
      if (new Date() - accessToken.date > 10000) {
        await this.UpdateToken();
      }
    } else {
      await this.UpdateToken();
    }
    let request = {
      mission_id: externalId,
      files: results.files,
      result: results.data,
    };
    logger.debug(`missionMedia request: ${JSON.stringify(request)}`);

    let sendResponse = await AppFetch(`${extAppUrl}/drones/mission/media`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${accessToken.token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });
    if (sendResponse.ok) {
      logger.info('missionMedia sent to external application successfully');
    } else {
      logger.error("error sending mission media to external application");
    }
  }
}
