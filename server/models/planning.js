import { writeYAML, readYAML, getRandomInt } from '../common/utils.js';

const configPlanning = readYAML('../config/elements/config.yaml');
var initPlanning = readYAML('../config/elements/mission_init.yaml');

export class planningModel {
  static getTypes() {
    console.log('Mission types');
    return configPlanning.missionTypes.map((mission) => ({
      id: mission.id,
      name: mission.name,
      type: mission.type,
      description: mission.description,
    }));
  }
  static getParam(type) {
    console.log('Mission types');
    return configPlanning.missionTypes[type]['setting'];
  }
  static getBasesSettings() {
    console.log('Get bases settingd');
    return initPlanning.bases;
  }
  static getMissionTypes() {
    console.log('Mission types');
    return configPlanning.missionTypes;
  }
  static getDefault() {
    console.log('Get default planning with markers');
    return { ...initPlanning, id: getRandomInt(100000000) };
  }
  static getPlanning() {
    console.log('Get default planing ');
    return {
      name: initPlanning.name,
      objetivo: initPlanning.objetivo,
      loc: initPlanning.loc,
      meteo: initPlanning.meteo,
      bases: initPlanning.bases,
      settings: initPlanning.settings,
    };
  }
  static setDefault(value) {
    console.log('Set default mission');
    console.log(value);
    initPlanning = value;
    let response = writeYAML('../config/elements/mission_init.yaml', value);
    return { result: response };
    // modify the mission init
  }

  static setMarkers(value) {
    // modify de markers
    console.log('Set markers');
    auxinitPlanning = { ...initPlanning, markersbase: value.markersbase, elements: value.elements };
    initPlanning = auxinitPlanning;
    let response = writeYAML('../config/elements/mission_init.yaml', auxinitPlanning);
    return { result: response };
  }
  static getMarkers() {
    console.log('get markers');
    return { markersbase: initPlanning.markersbase, elements: initPlanning.elements };
  }
  static getBases() {
    console.log('get Bases');
    return initPlanning.markersbase;
  }
  static getElements() {
    console.log('get elements');
    return initPlanning.elements;
  }
}
