import { LocalGlyphs } from '../config/config.js';
import * as ServerSetting from '../config/config.js';
import { checkFile } from '../common/utils.js';
export class serverModel {
  static DateTime() {
    const dateObject = new Date();
    let stringdate = dateObject.toJSON(); //dateObject.toJSON().slice(0, -1).replace('T',' ');
    console.log('uav sincronize time' + stringdate);
    return { datetime: stringdate };
  }
  static Serverconfig() {
    console.log('Get server config');
    console.log(ServerSetting);
    return {
      id: 1,
      attributes: {
        speedUnit: 'kmh',
        speedLimit: 37.796976241900644,
        timezone: 'Spain/Madrid',
        distanceUnit: 'km',
        glyphs: LocalGlyphs,
      },
      rosState: false,
      registration: true,
      readonly: false,
      deviceReadonly: false,
      map: 'osm',
      bingKey: '',
      mapUrl: '',
      overlayUrl: null,
      latitude: 0.0,
      longitude: 0.0,
      zoom: 0,
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
