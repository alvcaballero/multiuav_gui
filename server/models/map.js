import * as turf from '@turf/turf';
import { NoElevation } from '../config/config.js';

// Constantes de configuración
const ELEVATION_API_URL = 'https://api.opentopodata.org/v1/eudem25m';
const MAX_LOCATIONS_PER_REQUEST = 99;
const MIN_DISTANCE_FOR_INTERPOLATION_METERS = 200;

export class mapModel {
  /**
   * Divide una lista de ubicaciones en chunks para respetar el límite de la API
   * @param {Array} locations - Lista de coordenadas [lat, lng]
   * @returns {Array} Lista de chunks
   */
  static splitLocationsIntoChunks(locations) {
    const chunks = [];
    for (let i = 0; i < locations.length; i += MAX_LOCATIONS_PER_REQUEST) {
      chunks.push(locations.slice(i, i + MAX_LOCATIONS_PER_REQUEST));
    }
    return chunks;
  }

  /**
   * Crea una respuesta de elevación con valor cero para una lista de ubicaciones
   * @param {Array} locations - Lista de coordenadas [lat, lng]
   * @returns {Array} Lista de resultados con elevación 0
   */
  static createZeroElevationResults(locations) {
    return locations.map((coord) => ({
      location: { lat: coord[0], lng: coord[1] },
      elevation: 0,
    }));
  }

  /**
   * Formatea una lista de ubicaciones para la URL de la API
   * @param {Array} locations - Lista de coordenadas [lat, lng]
   * @returns {string} Cadena formateada "lat,lng|lat,lng|..."
   */
  static formatLocationsForApi(locations) {
    return locations.map((coord) => coord.join(',')).join('|');
  }

  /**
   * Obtiene datos de elevación desde la API externa
   * @param {Array} locationList - Lista de coordenadas [lat, lng]
   * @returns {Object} Respuesta con status y results
   */
  static async ApiElevation(locationList) {
    if (!locationList || locationList.length === 0) {
      return { status: 'ok', results: [] };
    }

    const chunks = this.splitLocationsIntoChunks(locationList);

    // Si la API de elevación está deshabilitada, devolver elevaciones en cero
    if (NoElevation) {
      console.log('Elevation API disabled - returning zero elevations');
      const allResults = chunks.flatMap((chunk) => this.createZeroElevationResults(chunk));
      return { status: 'warning', results: allResults };
    }

    console.log(`Fetching elevation data for ${locationList.length} points in ${chunks.length} request(s)`);

    const allResults = [];
    let hasErrors = false;

    for (const chunk of chunks) {
      const queryString = this.formatLocationsForApi(chunk);

      try {
        const response = await fetch(`${ELEVATION_API_URL}?locations=${queryString}`);
        const data = await response.json();

        if (data.results) {
          allResults.push(...data.results);
        } else {
          console.warn('Elevation API returned no results for chunk');
          allResults.push(...this.createZeroElevationResults(chunk));
          hasErrors = true;
        }
      } catch (error) {
        console.error(`Elevation API fetch error: ${error.message}`);
        allResults.push(...this.createZeroElevationResults(chunk));
        hasErrors = true;
      }
    }

    return {
      status: hasErrors ? 'warning' : 'ok',
      results: allResults,
    };
  }

  /**
   * Calcula la distancia entre dos waypoints usando turf.js
   * @param {Array} wp1 - Primer waypoint [lat, lng, alt]
   * @param {Array} wp2 - Segundo waypoint [lat, lng, alt]
   * @returns {number} Distancia en metros
   */
  static calculateDistanceBetweenWaypoints(wp1, wp2) {
    // Convertir de [lat, lng, alt] a [lng, lat] para turf.js (formato GeoJSON)
    const point1 = [wp1[1], wp1[0]];
    const point2 = [wp2[1], wp2[0]];
    const line = turf.lineString([point1, point2]);
    return turf.length(line, { units: 'meters' });
  }

  /**
   * Calcula la longitud acumulada de una ruta hasta un punto dado
   * @param {Array} waypoints - Lista de waypoints acumulados [lat, lng, alt]
   * @returns {number} Longitud total en metros
   */
  static calculateAccumulatedLength(waypoints) {
    if (waypoints.length < 2) return 0;
    // Convertir de [lat, lng, alt] a [lng, lat] para turf.js (formato GeoJSON)
    const geoJsonCoords = waypoints.map((wp) => [wp[1], wp[0]]);
    const line = turf.lineString(geoJsonCoords);
    return turf.length(line, { units: 'meters' });
  }

  /**
   * Procesa un segmento largo interpolando puntos intermedios
   * @param {Array} previousWp - Waypoint anterior
   * @param {Array} currentWp - Waypoint actual
   * @param {number} distance - Distancia entre waypoints
   * @param {number} lastAccumulatedDistance - Distancia acumulada hasta el punto anterior
   * @returns {Object} Objeto con puntos para la lista de waypoints y datos de altitud
   */
  static interpolateLongSegment(previousWp, currentWp, distance, lastAccumulatedDistance) {
    const steps = Math.floor(distance / MIN_DISTANCE_FOR_INTERPOLATION_METERS) + 1;
    const interpolatedPoints = this.interpolatePointsAlongLine([previousWp, currentWp], steps, distance);

    const waypointCoords = [];
    const altitudeData = [];

    for (const point of interpolatedPoints) {
      waypointCoords.push([point.lat, point.lon]);
      const accumulatedDist = Number((lastAccumulatedDistance + point.dist).toFixed(1));
      altitudeData.push({
        length: accumulatedDist,
        uav: null, // Puntos interpolados no tienen altitud de UAV definida
      });
    }

    return { waypointCoords, altitudeData };
  }

  /**
   * Procesa una ruta individual extrayendo waypoints y calculando distancias
   * @param {Array} route - Lista de waypoints de la ruta [lat, lng, alt]
   * @returns {Object} Objeto con coordenadas y datos de altitud
   */
  static processRoute(route) {
    const waypointCoords = [];
    const altitudeData = [];
    const accumulatedWaypoints = [];
    let lastWaypointIndex = 0;
    let lastAccumulatedDistance = 0;

    for (let index = 0; index < route.length; index++) {
      const waypoint = route[index];
      const altitude = waypoint[2];
      accumulatedWaypoints.push(waypoint);

      let currentLength = 0;

      if (index > 0) {
        currentLength = this.calculateAccumulatedLength(accumulatedWaypoints);
        const previousWaypoint = route[lastWaypointIndex];
        const segmentDistance = this.calculateDistanceBetweenWaypoints(previousWaypoint, waypoint);

        // Si el segmento es largo, interpolar puntos intermedios para mejor resolución de elevación
        if (segmentDistance > MIN_DISTANCE_FOR_INTERPOLATION_METERS) {
          const { waypointCoords: interpolatedCoords, altitudeData: interpolatedAltitudes } = this.interpolateLongSegment(
            previousWaypoint,
            waypoint,
            segmentDistance,
            lastAccumulatedDistance
          );
          waypointCoords.push(...interpolatedCoords);
          altitudeData.push(...interpolatedAltitudes);
        }
      }

      // Añadir el waypoint actual
      waypointCoords.push([waypoint[0], waypoint[1]]);
      altitudeData.push({
        length: Number(currentLength.toFixed(1)),
        uav: altitude,
      });

      lastWaypointIndex = index;
      lastAccumulatedDistance = Number(currentLength.toFixed(1));
    }

    return { waypointCoords, altitudeData };
  }

  /**
   * Enriquece los datos de altitud con información de elevación del terreno
   * @param {Array} altitudeDataByRoute - Datos de altitud organizados por ruta
   * @param {Object} elevationProfile - Respuesta de la API de elevación
   * @returns {Array} Datos enriquecidos con elevación
   */
  static enrichWithElevationData(altitudeDataByRoute, elevationProfile) {
    if (!elevationProfile?.results || elevationProfile.results.length === 0) {
      console.error('No elevation results available');
      throw new Error('No elevation data available');
    }

    let elevationIndex = 0;

    for (let routeIndex = 0; routeIndex < altitudeDataByRoute.length; routeIndex++) {
      const routeData = altitudeDataByRoute[routeIndex];
      let waypointCounter = 0;
      let routeStartElevation = 0;

      for (let pointIndex = 0; pointIndex < routeData.length; pointIndex++) {
        const elevationResult = elevationProfile.results[elevationIndex];

        if (!elevationResult) {
          console.error(`Missing elevation data at index ${elevationIndex}`);
          throw new Error(`Missing elevation data at index ${elevationIndex}`);
        }

        const point = routeData[pointIndex];

        // Añadir datos de elevación del terreno
        point.elevation = Number(elevationResult.elevation).toFixed(1);
        point.lat = Number(elevationResult.location.lat);
        point.lng = Number(elevationResult.location.lng);
        point.rt = routeIndex;

        // Guardar la elevación inicial de la ruta para calcular altura absoluta
        if (pointIndex === 0) {
          routeStartElevation = Number(elevationResult.elevation);
        }

        // Si es un waypoint real (no interpolado), calcular altura absoluta del UAV
        if (point.uav !== null) {
          point.wp = waypointCounter;
          point.uavheight = (Number(point.uav) + routeStartElevation).toFixed(1);
          waypointCounter++;
        }

        elevationIndex++;
      }
    }

    return altitudeDataByRoute;
  }

  /**
   * Calcula el perfil de elevación para un conjunto de rutas
   * @param {Array} routes - Lista de rutas, cada una con waypoints [lat, lng, alt]
   * @returns {Object} Objeto con elevation (datos de elevación) y status (éxito/fallo)
   */
  static async calcElevation(routes) {
    if (!routes || routes.length === 0) {
      return { elevation: [], status: true };
    }

    console.log(`Calculating elevation profile for ${routes.length} route(s)`);

    // Paso 1: Procesar todas las rutas y extraer waypoints
    const allWaypointCoords = [];
    const altitudeDataByRoute = [];

    for (const route of routes) {
      const { waypointCoords, altitudeData } = this.processRoute(route);
      allWaypointCoords.push(...waypointCoords);
      altitudeDataByRoute.push(altitudeData);
    }

    console.log(`Total waypoint coords: ${allWaypointCoords.length}, Routes processed: ${altitudeDataByRoute.length}`);

    // Paso 2: Obtener elevación del terreno para todos los puntos
    const elevationProfile = await this.ApiElevation(allWaypointCoords);

    console.log(`Elevation API returned ${elevationProfile?.results?.length || 0} results`);

    // Paso 3: Enriquecer datos con elevación del terreno
    try {
      const enrichedData = this.enrichWithElevationData(altitudeDataByRoute, elevationProfile);
      console.log(`Elevation calculation successful, returning ${enrichedData.length} routes`);
      return { elevation: enrichedData, status: true };
    } catch (error) {
      console.error(`Error processing elevation profile: ${error.message}`);
      return { elevation: [], status: false };
    }
  }

  /**
   * Interpola puntos a lo largo de una línea entre dos waypoints
   * Genera puntos intermedios equidistantes (excluyendo inicio y fin)
   * @param {Array} line - Array de dos puntos [inicio, fin], cada uno [lat, lng, ...]
   * @param {number} steps - Número de segmentos en los que dividir la línea
   * @param {number} totalDistance - Distancia total de la línea en metros
   * @returns {Array} Lista de puntos interpolados con {lat, lon, dist}
   */
  static interpolatePointsAlongLine(line, steps, totalDistance) {
    const startPoint = line[0];
    const endPoint = line[1];

    const latStep = (endPoint[0] - startPoint[0]) / steps;
    const lngStep = (endPoint[1] - startPoint[1]) / steps;
    const distStep = totalDistance / steps;

    const interpolatedPoints = [];

    // Generar puntos intermedios (excluyendo el punto inicial y final)
    for (let i = 1; i < steps; i++) {
      interpolatedPoints.push({
        lat: startPoint[0] + i * latStep,
        lon: startPoint[1] + i * lngStep,
        dist: i * distStep,
      });
    }

    return interpolatedPoints;
  }
}
