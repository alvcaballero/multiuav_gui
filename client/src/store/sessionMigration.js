/**
 * Funciones helper para migrar datos de la estructura legacy a la nueva estructura
 */

/**
 * Genera un ID local para un grupo/item de elementos creado en el cliente
 * antes de guardarse. Prefijado con "local_" para distinguirlo de un id real
 * de SQL (siempre numérico) sin ambigüedad.
 */
export const generateLocalId = () => {
  return `local_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
};

/**
 * Migra markers de estructura legacy a nueva estructura.
 * Añade IDs a las bases y a los grupos/items de elements si no los tienen
 * (el backend siempre los manda poblados; esto solo cubre un grupo/item
 * recién creado en el cliente o un snapshot cargado desde un YAML local).
 */
export const migrateMarkers = (markers) => {
  if (!markers) {
    return markers;
  }

  // Una base con `id` (real, asignado por el servidor) se deja intacta. Una
  // base recién creada en el cliente, todavía sin guardar, recibe un
  // `tempId` (nunca `id`) para que React tenga una key estable sin que
  // parezca jamás un id real persistido — se descarta antes de que la base
  // exista en SQL, el servidor siempre asigna el `id` real en el primer guardado.
  const migratedBases = (markers.bases || []).map((base) => {
    if (base.id != null) {
      return base;
    }
    return {
      ...base,
      tempId: base.tempId ?? generateLocalId(),
    };
  });

  const migratedElements = (markers.elements || []).map((group) => {
    const groupId = group.groupId ?? generateLocalId();
    const items = (group.items || []).map((item) => ({
      ...item,
      itemId: item.itemId ?? generateLocalId(),
      groupId: item.groupId ?? groupId,
    }));
    return { ...group, groupId, items };
  });

  return {
    ...markers,
    bases: migratedBases,
    elements: migratedElements,
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
    (b) => b.devices && b.devices.id && b.devices.id !== '',
  );
  if (firstValidBase && firstValidBase.settings) {
    defaultSettings = { ...firstValidBase.settings };
  }

  // Crear assignments solo para bases que tienen dispositivos asignados
  const assignments = planning.bases.flatMap((base, index) => {
    // Saltar entradas vacías
    if (!base.devices || !base.devices.id || base.devices.id === '') {
      return [];
    }

    // Obtener el baseId correspondiente del array de markers
    const baseId = markers.bases[index]?.id;
    if (!baseId) {
      console.warn(`No se encontró base en markers para índice ${index}`);
      return [];
    }

    return [
      {
        baseId,
        device: {
          id: String(base.devices.id), // Normalizar a string
          name: base.devices.name || '',
        },
        settings: { ...base.settings },
      },
    ];
  });

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
