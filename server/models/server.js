import { LocalGlyphs, mapLatitude, mapLongitude, mapZoom, LLM } from '../config/config.js';
import * as ServerSetting from '../config/config.js';
import { checkFile, readDataFile } from '../common/utils.js';
import { logger } from '../common/logger.js';

const buildingImages = readDataFile('../data/elements.json');
export class serverModel {
  static DateTime() {
    const dateObject = new Date();
    let stringdate = dateObject.toJSON(); //dateObject.toJSON().slice(0, -1).replace('T',' ');
    logger.debug(`UAV synchronize time: ${stringdate}`);
    return { datetime: stringdate };
  }
  static Serverconfig() {
    return {
      id: 1,
      attributes: {
        speedUnit: 'kmh',
        speedLimit: 37.796976241900644,
        timezone: 'Spain/Madrid',
        distanceUnit: 'km',
        mapLiveRoutes: 'yes',
        'web.liveRouteLength': 100,
        glyphs: LocalGlyphs,
        customElements: buildingImages.hasOwnProperty('elements') ? buildingImages.elements : [],
      },
      rosState: false,
      registration: true,
      readonly: false,
      deviceReadonly: false,
      map: 'osm',
      bingKey: '',
      mapUrl: '',
      overlayUrl: null,
      latitude: mapLatitude,
      longitude: mapLongitude,
      zoom: mapZoom,
      twelveHourFormat: false,
      forceSettings: false,
      coordinateFormat: '',
      limitCommands: false,
      disableReports: false,
      fixedEmail: false,
      poiLayer: '',
      announcement: '',
      emailEnabled: true,
      geocoderEnabled: true,
      textEnabled: false,
      storageSpace: [1646120960, 12293705728],
      newServer: false,
      openIdEnabled: false,
      openIdForce: false,
      llmEnabled: LLM,
    };
  }
  static Protocol() {
    let protocol = [];
    ServerSetting.RosEnable ? protocol.push('ros') : null;
    ServerSetting.FbEnable ? protocol.push('robofleet') : null;
    return protocol;
  }

  static checkFileRoute(path) {
    return checkFile('../resources/' + path.replaceAll('-', '/'));
  }
}
