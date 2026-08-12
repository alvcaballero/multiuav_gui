/**
 * Selectors para acceder a los datos de session de forma segura y consistente
 */
import { createSelector } from '@reduxjs/toolkit';

// ─── Inspection targets (elements) ───────────────────────────────────────────

/**
 * Obtiene todos los grupos de inspection targets
 */
export const getAllInspectionGroups = (state) => state.session.markers?.elements || [];

/**
 * Obtiene los items con imagen georreferenciada (corners definidos) listos para MapLibre.
 * Formato: [{ key, url, coordinates }]
 * Solo incluye items que tengan corners completos (4 puntos [lng, lat]).
 * Memoizado con createSelector para evitar re-renders innecesarios.
 */
export const getMapImageItems = createSelector(getAllInspectionGroups, (groups) =>
  groups.flatMap((group) => {
    let idx = 0;
    return (group.items || []).flatMap((item) => {
      if (!Array.isArray(item.corners) || item.corners.length !== 4) return [];
      const result = {
        key: `${group.type}-${idx}`,
        url: `/api/markers/types/${group.type}/icon`,
        coordinates: item.corners,
      };
      idx += 1;
      return [result];
    });
  }),
);
