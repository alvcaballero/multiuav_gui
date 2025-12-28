/**
 * Planning Service
 *
 * Contiene toda la lógica de negocio relacionada con planning y gestión de puntos.
 * Separada del componente UI para mejor testabilidad y reutilización.
 */

/**
 * Tipos de objetivo disponibles
 */
export const OBJETIVO_TYPES = {
  PATH_OBJECT: 'path-object',
  OBJECT: 'object',
  POINT: 'point',
};

/**
 * Tipos de elementos que se pueden crear
 */
export const ELEMENT_TYPES = {
  POWER_TOWER: 'powerTower',
  WIND_TURBINE: 'windTurbine',
};

/**
 * Gestiona la adición de puntos a la lista de localizaciones según el tipo de objetivo.
 *
 * @param {Array} locations - Array actual de localizaciones (no se muta)
 * @param {Object} point - Punto a agregar con {latitude, longitude, groupId?}
 * @param {string} objetivoType - Tipo de objetivo ('path-object', 'object', 'point')
 * @returns {Array} Nuevo array de localizaciones con el punto agregado
 */
export const manageLocationPoints = (locations, point, objetivoType) => {
  // Crear copia para no mutar el original
  const newLocations = [...locations];

  switch (objetivoType) {
    case OBJETIVO_TYPES.PATH_OBJECT:
      return managePathObjectPoints(newLocations, point);

    case OBJETIVO_TYPES.OBJECT:
      return manageObjectPoints(newLocations, point);

    case OBJETIVO_TYPES.POINT:
      return managePointPoints(newLocations, point);

    default:
      console.warn(`Unknown objetivo type: ${objetivoType}`);
      return newLocations;
  }
};

/**
 * Gestiona puntos para objetivos de tipo 'path-object'
 * Agrupa puntos por groupId en el mismo elemento
 *
 * @private
 */
const managePathObjectPoints = (locations, point) => {
  // Si no hay localizaciones, crear la primera
  if (locations.length === 0) {
    return [createNewElement(point)];
  }

  // Buscar si ya existe un elemento con el mismo groupId
  const existingRouteIndex = locations.findIndex(
    (element) => element.items[0]?.groupId === point.groupId
  );

  if (existingRouteIndex === -1) {
    // No existe, crear nuevo elemento
    return [...locations, createNewElement(point)];
  } else {
    // Ya existe, agregar punto al elemento existente
    const updatedLocations = [...locations];
    updatedLocations[existingRouteIndex] = {
      ...updatedLocations[existingRouteIndex],
      items: [...updatedLocations[existingRouteIndex].items, point],
    };
    return updatedLocations;
  }
};

/**
 * Gestiona puntos para objetivos de tipo 'object'
 * Cada punto crea un nuevo elemento
 *
 * @private
 */
const manageObjectPoints = (locations, point) => {
  return [...locations, createNewElement(point)];
};

/**
 * Gestiona puntos para objetivos de tipo 'point'
 * Cada punto crea un nuevo elemento
 *
 * @private
 */
const managePointPoints = (locations, point) => {
  return [...locations, createNewElement(point)];
};

/**
 * Crea un nuevo elemento de tipo powerTower
 *
 * @private
 * @param {Object} point - Punto inicial del elemento
 * @returns {Object} Nuevo elemento con estructura estándar
 */
const createNewElement = (point) => {
  return {
    type: ELEMENT_TYPES.POWER_TOWER,
    name: 'Elements',
    linea: true,
    items: [point],
  };
};

/**
 * Valida que un punto tenga la estructura correcta
 *
 * @param {Object} point - Punto a validar
 * @returns {boolean} True si el punto es válido
 */
export const isValidPoint = (point) => {
  return (
    point &&
    typeof point === 'object' &&
    typeof point.latitude === 'number' &&
    typeof point.longitude === 'number'
  );
};

/**
 * Valida que un array de locations tenga la estructura correcta
 *
 * @param {Array} locations - Array de locations a validar
 * @returns {boolean} True si todas las locations son válidas
 */
export const areValidLocations = (locations) => {
  if (!Array.isArray(locations)) return false;

  return locations.every(
    (location) =>
      location &&
      typeof location === 'object' &&
      location.type &&
      Array.isArray(location.items) &&
      location.items.every(isValidPoint)
  );
};

/**
 * Transforma locations a formato de API
 *
 * @param {Array} locations - Array de locations
 * @returns {Array} Locations transformadas para la API
 */
export const transformLocationsForAPI = (locations) => {
  return locations.map((group) => ({
    name: group.name,
    items: group.items.map((element) => ({
      latitude: element.latitude,
      longitude: element.longitude,
    })),
  }));
};

/**
 * Valida que no haya dispositivos duplicados en assignments
 *
 * @param {Array} assignments - Array de assignments
 * @returns {Object} { isValid: boolean, duplicates: Array, errorMsg: string }
 */
export const validateUniqueDevices = (assignments) => {
  const deviceIds = assignments
    .map((assignment) => assignment.device.id)
    .filter((id) => id !== '');

  const hasDuplicates = deviceIds.some((id, index, list) => list.indexOf(id) !== index);

  if (hasDuplicates) {
    const duplicates = deviceIds.filter((id, index, list) => list.indexOf(id) !== index);
    const uniqueDuplicates = [...new Set(duplicates)];

    return {
      isValid: false,
      duplicates: uniqueDuplicates,
      errorMsg: `Device(s) repeated: ${uniqueDuplicates.join(', ')}. Please assign different devices to each base.`,
    };
  }

  return {
    isValid: true,
    duplicates: [],
    errorMsg: '',
  };
};
