import { LocalGlyphs } from '../config/config.js';
import * as ServerSetting from '../config/config.js';
import { checkFile } from '../common/utils.js';
import { readDataFile } from '../common/utils.js';

const buildingImages = readDataFile('../data/elements.json');
export class serverModel {
  static DateTime() {
    const dateObject = new Date();
    let stringdate = dateObject.toJSON(); //dateObject.toJSON().slice(0, -1).replace('T',' ');
    console.log('uav sincronize time' + stringdate);
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
      latitude: 41.68734389317842, //37.19374
      longitude: -8.84768639812097, //-6.702911,
      zoom: 12,
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
