/**
 * File Service
 *
 * Lógica de I/O y parseo de archivos, desacoplada de componentes UI.
 * Las funciones retornan datos; el llamador es responsable de despachar acciones Redux.
 */
import YAML from 'yaml';

/**
 * Lee un archivo como texto y ejecuta un callback con el contenido.
 *
 * @param {File} file
 * @param {(result: { name: string, data: string }) => void} onLoad
 * @param {(error: DOMException) => void} [onError]
 */
export const readTextFile = (file, onLoad, onError) => {
  if (!file) return;
  const reader = new FileReader();
  reader.readAsText(file);
  reader.onload = () => onLoad({ name: file.name, data: reader.result });
  reader.onerror = () => {
    console.error('FileReader error:', reader.error);
    onError?.(reader.error);
  };
};

/**
 * Resultado del parseo de un KML de elementos.
 *
 * @typedef {Object} KmlElementsResult
 * @property {'bases' | 'towers'} kind  - Qué tipo de elementos se encontraron
 * @property {Array<Object>} markers    - Array listo para despachar al store
 */

/**
 * Parsea un KML de elementos y retorna los marcadores clasificados.
 * El KML puede contener <Point> (bases) o <coordinates> (torres/líneas).
 *
 * @param {string} kmlText - Contenido XML en texto
 * @returns {KmlElementsResult | null} null si el KML no contiene datos reconocibles
 */
export const parseKmlElements = (kmlText) => {
  const xmlDocument = new DOMParser().parseFromString(kmlText, 'text/xml');

  // --- Caso 1: <Point> → marcadores de base ---
  const pointNodes = xmlDocument.getElementsByTagName('Point');
  const pointArray = Object.values(pointNodes).map((node) =>
    node.textContent
      .replace('\t1', '')
      .replace(/(\r\n|\n|\r|\t)/gm, '')
      .split(','),
  );

  if (pointArray.length) {
    const markers = pointArray.map((coords) => ({
      latitude: Number(coords[1]),
      longitude: Number(coords[0]),
      image: 'base',
    }));
    return { kind: 'bases', markers };
  }

  // --- Caso 2: <coordinates> → torres de potencia ---
  const coordNodes = xmlDocument.getElementsByTagName('coordinates');
  const coordGroups = Object.values(coordNodes).map((node) =>
    node.textContent
      .replace('\t1', '')
      .replace(/(\r\n|\n|\r|\t)/gm, '')
      .split(' ')
      .map((pair) => pair.split(',')),
  );

  if (coordGroups.length) {
    const markers = coordGroups.map((group) => ({
      type: 'powerTower',
      items: group.flatMap((coords) =>
        coords.length > 1
          ? [
              {
                latitude: Number(coords[1]),
                longitude: Number(coords[0]),
              },
            ]
          : [],
      ),
    }));
    return { kind: 'towers', markers };
  }

  return null;
};

// ---------------------------------------------------------------------------
// Mission file parsers
// ---------------------------------------------------------------------------

/**
 * Parsea un archivo de misión y retorna los datos listos para el store.
 * No despacha nada — responsabilidad del llamador.
 *
 * @param {{ name: string, data: string }} item
 * @returns {{ mission: Object, name: string } | null} null si el formato no es soportado
 */
export const parseMissionFile = ({ name, data }) => {
  let cleanName = name;
  let mission;

  if (name.endsWith('.yaml') || name.endsWith('.yml')) {
    cleanName = name.replace(/\.ya?ml$/, '');
    mission = YAML.parse(data);
  } else if (name.endsWith('.waypoints')) {
    cleanName = name.replace('.waypoints', '');
    mission = parseWaypointFile(data);
  } else if (name.endsWith('.kml')) {
    cleanName = name.replace('.kml', '');
    mission = parseKmlMission(data);
  } else if (name.endsWith('.plan')) {
    cleanName = name.replace('.plan', '');
    mission = parsePlanFile(data);
  } else {
    return null;
  }

  return { mission, name: cleanName };
};

// --- parsers privados -------------------------------------------------------

const parsePlanFile = (data) => {
  const jsondoc = JSON.parse(data);
  const mission_yaml = { uav_n: 1, uav_1: {} };
  let count_wp = 0;
  jsondoc.mission.items.forEach((element) => {
    mission_yaml.uav_1['wp_' + count_wp] = [element.params[4], element.params[5], element.Altitude];
    count_wp++;
  });
  mission_yaml.uav_1['wp_n'] = count_wp;
  return mission_yaml;
};

const parseKmlMission = (data) => {
  const xmlDocument = new DOMParser().parseFromString(data, 'text/xml');
  const missionxml = xmlDocument.getElementsByTagName('coordinates');
  const routes = Object.values(missionxml).map((node) =>
    node.textContent
      .replace('\t1', '')
      .replace(/(\r\n|\n|\r|\t)/gm, '')
      .split(' ')
      .map((point) => point.split(',')),
  );

  const mission_yaml = { uav_n: routes.length };
  routes.forEach((route, idx) => {
    const uavKey = 'uav_' + (idx + 1);
    mission_yaml[uavKey] = {};
    let count_wp = 0;
    route.forEach((element) => {
      if (element.length === 3) {
        mission_yaml[uavKey]['wp_' + count_wp] = [element[1], element[0], element[2]];
        count_wp++;
      }
    });
    mission_yaml[uavKey]['wp_n'] = count_wp;
  });

  return mission_yaml;
};

const parseWaypointFile = (data) => {
  const mission_yaml = { uav_n: 1, uav_1: {} };
  let count_wp = 0;
  data.split('\n').forEach((line) => {
    const cols = line.split('\t');
    if (cols[3] === '16') {
      mission_yaml.uav_1['wp_' + count_wp] = [cols[8], cols[9], cols[10]];
      count_wp++;
    }
  });
  mission_yaml.uav_1['wp_n'] = count_wp;
  return mission_yaml;
};
