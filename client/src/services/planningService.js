/**
 * Planning Service
 *
 * Contiene toda la lógica de negocio relacionada con planning y gestión de puntos.
 * Separada del componente UI para mejor testabilidad y reutilización.
 */

/**
 * Tipos de objetivo disponibles
 */
const OBJETIVO_TYPES = {
  PATH_OBJECT: 'path-object',
  OBJECT: 'object',
  POINT: 'point',
};

/**
 * Gestiona la adición de puntos a la lista de localizaciones según el tipo de objetivo.
 *
 * @param {Array} locations - Array actual de localizaciones (no se muta)
 * @param {Object} point - Punto a agregar con {latitude, longitude, groupId?}
 * @param {string} objetivoType - Tipo de objetivo ('path-object', 'object', 'point')
 * @param {number} powerTowerTypeId - id del ElementType 'Power Tower' (catálogo dinámico, ver useMarkerTypes)
 * @returns {Array} Nuevo array de localizaciones con el punto agregado
 */
export const manageLocationPoints = (locations, point, objetivoType, powerTowerTypeId) => {
  // Crear copia para no mutar el original
  const newLocations = [...locations];

  switch (objetivoType) {
    case OBJETIVO_TYPES.PATH_OBJECT:
      return managePathObjectPoints(newLocations, point, powerTowerTypeId);

    case OBJETIVO_TYPES.OBJECT:
      return manageObjectPoints(newLocations, point, powerTowerTypeId);

    case OBJETIVO_TYPES.POINT:
      return managePointPoints(newLocations, point, powerTowerTypeId);

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
const managePathObjectPoints = (locations, point, powerTowerTypeId) => {
  // Si no hay localizaciones, crear la primera
  if (locations.length === 0) {
    return [createNewElement(point, powerTowerTypeId)];
  }

  // Buscar si ya existe un elemento con el mismo groupId
  const existingRouteIndex = locations.findIndex(
    (element) => element.items[0]?.groupId === point.groupId,
  );

  if (existingRouteIndex === -1) {
    // No existe, crear nuevo elemento
    return [...locations, createNewElement(point, powerTowerTypeId)];
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
const manageObjectPoints = (locations, point, powerTowerTypeId) => {
  return [...locations, createNewElement(point, powerTowerTypeId)];
};

/**
 * Gestiona puntos para objetivos de tipo 'point'
 * Cada punto crea un nuevo elemento
 *
 * @private
 */
const managePointPoints = (locations, point, powerTowerTypeId) => {
  return [...locations, createNewElement(point, powerTowerTypeId)];
};

/**
 * Crea un nuevo elemento de tipo Power Tower
 *
 * @private
 * @param {Object} point - Punto inicial del elemento
 * @param {number} powerTowerTypeId - id del ElementType 'Power Tower'
 * @returns {Object} Nuevo elemento con estructura estándar
 */
const createNewElement = (point, powerTowerTypeId) => {
  return {
    type: powerTowerTypeId,
    name: 'Elements',
    linea: true,
    items: [point],
  };
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
  const deviceIds = assignments.flatMap((assignment) =>
    assignment.device.id !== '' ? [assignment.device.id] : [],
  );

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
