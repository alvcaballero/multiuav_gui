/**
 * Helpers geoespaciales compartidos.
 *
 * Dos formas de medir distancia horizontal entre coordenadas geográficas, con
 * distinto compromiso precisión/costo. Elegí según el caso:
 *
 *  - `haversineMeters`: fórmula haversine exacta. Úsala cuando la precisión
 *    importa (tracking de waypoints, detección de desvío).
 *  - `approxDistanceMeters`: aproximación equirectangular (proyección plana).
 *    Más barata; error despreciable a escalas de metros/decenas de metros.
 *    Úsala en caminos calientes donde solo comparás contra un umbral (dead-band).
 */

const EARTH_RADIUS_M = 6_371_000;
const DEG_TO_RAD = Math.PI / 180;

/**
 * Distancia haversine exacta entre dos puntos (metros).
 */
export function haversineMeters(lat1, lon1, lat2, lon2) {
  const dLat = (lat2 - lat1) * DEG_TO_RAD;
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * DEG_TO_RAD) * Math.cos(lat2 * DEG_TO_RAD) * Math.sin(dLon / 2) ** 2;
  return EARTH_RADIUS_M * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

/**
 * Distancia horizontal aproximada (equirectangular) entre dos puntos (metros).
 * Suficiente para comparar contra umbrales a escalas de inspección.
 */
export function approxDistanceMeters(lat1, lon1, lat2, lon2) {
  const dLat = (lat2 - lat1) * DEG_TO_RAD;
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const meanLat = ((lat1 + lat2) / 2) * DEG_TO_RAD;
  const x = dLon * Math.cos(meanLat);
  const y = dLat;
  return Math.sqrt(x * x + y * y) * EARTH_RADIUS_M;
}
