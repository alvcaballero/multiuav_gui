/**
 * Selectors para acceder a los datos de session de forma segura y consistente
 */

/**
 * Obtiene todas las bases con sus asignaciones
 */
export const getAllBasesWithAssignments = (state) => {
  const markers = state.session.markers;
  const planning = state.session.planning;

  if (!markers?.bases || !planning?.assignments) {
    return [];
  }

  return markers.bases.map((base) => {
    const assignment = planning.assignments.find((a) => a.baseId === base.id);
    return {
      ...base,
      device: assignment?.device || null,
      settings: assignment?.settings || planning.defaultSettings || {},
    };
  });
};

/**
 * Obtiene una base específica por ID
 */
export const getBaseById = (state, baseId) => {
  const base = state.session.markers?.bases?.find((b) => b.id === baseId);
  if (!base) return null;

  const assignment = state.session.planning?.assignments?.find((a) => a.baseId === baseId);
  return {
    ...base,
    device: assignment?.device || null,
    settings: assignment?.settings || state.session.planning?.defaultSettings || {},
  };
};

/**
 * Obtiene la asignación para una base específica
 */
export const getAssignmentForBase = (state, baseId) => {
  return state.session.planning?.assignments?.find((a) => a.baseId === baseId);
};

/**
 * Obtiene todas las bases que tienen dispositivos asignados
 */
export const getAssignedBases = (state) => {
  const assignments = state.session.planning?.assignments || [];
  const bases = state.session.markers?.bases || [];

  return assignments
    .map((assignment) => {
      const base = bases.find((b) => b.id === assignment.baseId);
      if (!base) return null;
      return {
        ...base,
        device: assignment.device,
        settings: assignment.settings,
      };
    })
    .filter(Boolean);
};

/**
 * Obtiene las bases sin dispositivos asignados
 */
export const getUnassignedBases = (state) => {
  const assignments = state.session.planning?.assignments || [];
  const bases = state.session.markers?.bases || [];

  const assignedBaseIds = new Set(assignments.map((a) => a.baseId));

  return bases.filter((base) => !assignedBaseIds.has(base.id));
};

/**
 * Verifica si un dispositivo ya está asignado a alguna base
 */
export const isDeviceAssigned = (state, deviceId) => {
  const assignments = state.session.planning?.assignments || [];
  return assignments.some((a) => a.device?.id === deviceId);
};

/**
 * Obtiene el índice de una base dado su ID (útil para compatibilidad con código legacy)
 */
export const getBaseIndexById = (state, baseId) => {
  const bases = state.session.markers?.bases || [];
  return bases.findIndex((b) => b.id === baseId);
};

/**
 * Obtiene una base por su índice (útil para compatibilidad con código legacy)
 */
export const getBaseByIndex = (state, index) => {
  const bases = state.session.markers?.bases || [];
  return bases[index] || null;
};
