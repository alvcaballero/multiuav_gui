import { readJSON, readYAML } from '../common/utils.js';

const configPlanning = readYAML('../config/elements/config.yaml');
const initPlanning = readYAML('../config/elements/mission_init.yaml');

export class planningModel {
  static getTypes() {
    console.log('Mission types');
    return configPlanning.missionTypes.map((mission) => ({
      id: mission.id,
      name: mission.name,
      type: mission.type,
    }));
  }
  static getParam(type) {
    console.log('Mission types');
    return configPlanning.missionTypes[type]['setting'];
  }
  static getMissionTypes() {
    console.log('Mission types');
    return configPlanning.missionTypes;
  }
  static getDefault() {
    console.log('Get default mission');
    return {
      bases: initPlanning.bases,
      markers: initPlanning.markers,
      settings: initPlanning.settings,
    };
  }
  static setDefault(value) {
    console.log('Set default mission');
    console.log(value);
    return {};
    // modify the mission init
  }
  static setMarkers() {
    // modify de markers
    console.log('Set markers');
    console.log(value);
    return {};
  }
  static getBases() {
    console.log('get Bases');
    return initPlanning.bases;
  }
  static getElements() {
    console.log('get elements');
    return initPlanning.markers;
  }
}
