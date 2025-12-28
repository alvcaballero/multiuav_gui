/**
 * Funciones helper para migrar datos de la estructura legacy a la nueva estructura
 */

/**
 * Genera un ID único para una base
 */
export const generateBaseId = () => {
  return `base_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
};

/**
 * Migra markers de estructura legacy a nueva estructura
 * Añade IDs a las bases si no los tienen
 */
export const migrateMarkers = (markers) => {
  if (!markers || !markers.bases) {
    return markers;
  }

  const migratedBases = markers.bases.map((base, index) => {
    // Si ya tiene ID, no hacer nada
    if (base.id) {
      return base;
    }

    // Generar ID basado en el índice para mantener consistencia
    return {
      ...base,
      id: `base_${index}`,
    };
  });

  return {
    ...markers,
    bases: migratedBases,
  };
};

/**
 * Migra planning de estructura legacy (con array de bases por índice)
 * a nueva estructura (con assignments que referencian baseId)
 */
export const migratePlanning = (planning, markers) => {
  // Si no hay planning, retornar estructura por defecto
  if (!planning) {
    return {
      id: null,
      objetivo: {},
      loc: [],
      meteo: [],
      assignments: [],
      defaultSettings: {},
      settingsSchema: {},
    };
  }

  // Si ya tiene la nueva estructura (assignments), asegurar todas las propiedades
  if (planning.assignments) {
    // Normalizar assignments: asegurar que device.id sea string
    const normalizedAssignments = (planning.assignments || []).map((assignment) => ({
      ...assignment,
      device: {
        id: assignment.device?.id !== undefined ? String(assignment.device.id) : '',
        name: assignment.device?.name || '',
      },
    }));

    return {
      id: planning.id || null,
      objetivo: planning.objetivo || {},
      loc: planning.loc || [],
      meteo: planning.meteo || [],
      assignments: normalizedAssignments,
      defaultSettings: planning.defaultSettings || {},
      settingsSchema: planning.settingsSchema || {},
    };
  }

  // Si tiene estructura legacy pero no hay markers, no podemos migrar
  if (!planning.bases || !markers || !markers.bases) {
    return {
      id: planning.id || null,
      objetivo: planning.objetivo || {},
      loc: planning.loc || [],
      meteo: planning.meteo || [],
      assignments: [],
      defaultSettings: {},
      settingsSchema: planning.settings || {},
    };
  }

  // Extraer configuración por defecto del primer elemento no vacío
  let defaultSettings = {};
  const firstValidBase = planning.bases.find(
    (b) => b.devices && b.devices.id && b.devices.id !== ''
  );
  if (firstValidBase && firstValidBase.settings) {
    defaultSettings = { ...firstValidBase.settings };
  }

  // Crear assignments solo para bases que tienen dispositivos asignados
  const assignments = planning.bases
    .map((base, index) => {
      // Saltar entradas vacías
      if (!base.devices || !base.devices.id || base.devices.id === '') {
        return null;
      }

      // Obtener el baseId correspondiente del array de markers
      const baseId = markers.bases[index]?.id;
      if (!baseId) {
        console.warn(`No se encontró base en markers para índice ${index}`);
        return null;
      }

      return {
        baseId,
        device: {
          id: String(base.devices.id), // Normalizar a string
          name: base.devices.name || '',
        },
        settings: { ...base.settings },
      };
    })
    .filter(Boolean); // Remover nulls

  // Crear nueva estructura de planning
  const newPlanning = {
    id: planning.id,
    objetivo: planning.objetivo,
    loc: planning.loc || [],
    meteo: planning.meteo || [],
    assignments,
    defaultSettings,
    settingsSchema: planning.settings || {},
  };

  return newPlanning;
};

/**
 * Convierte la nueva estructura de planning a formato legacy
 * Útil para mantener compatibilidad con APIs existentes
 */
export const planningToLegacy = (planning, markers) => {
  if (!planning || !markers || !markers.bases) {
    return { bases: [] };
  }

  // Si ya está en formato legacy, retornar
  if (planning.bases && !planning.assignments) {
    return planning;
  }

  const assignments = planning.assignments || [];
  const defaultSettings = planning.defaultSettings || {};

  // Crear array de bases con el mismo tamaño que markers.bases
  const legacyBases = markers.bases.map((base) => {
    const assignment = assignments.find((a) => a.baseId === base.id);

    if (assignment) {
      // Base con asignación
      return {
        devices: {
          id: assignment.device.id,
          name: assignment.device.name,
        },
        settings: { ...assignment.settings },
      };
    } else {
      // Base sin asignación - usar valores vacíos
      return {
        devices: {
          id: '',
          name: '',
        },
        settings: { ...defaultSettings },
      };
    }
  });

  return {
    id: planning.id,
    objetivo: planning.objetivo,
    loc: planning.loc || [],
    meteo: planning.meteo || [],
    bases: legacyBases,
    settings: planning.settingsSchema || {},
  };
};

/**
 * Migra datos completos de session
 */
export const migrateSessionData = (session) => {
  if (!session) return session;

  const migratedMarkers = migrateMarkers(session.markers);
  const migratedPlanning = migratePlanning(session.planning, migratedMarkers);

  return {
    ...session,
    markers: migratedMarkers,
    planning: migratedPlanning,
  };
};
