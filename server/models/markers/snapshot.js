// Bridges the legacy YAML-shaped `{markersbase, elements, assignments}` blob
// (still what the client/planner speak) with the new SQL tables. Reads build
// that shape from ElementGroup/ElementItem/Base/Assignment; writes upsert the
// same three collections back into those tables. This is the seam the legacy
// `markersModel`/`planningModel` cut over to — the wire format they expose
// does not change, only where the data actually lives.
import sequelize from '../../common/sequelize.js';
import { logger } from '../../common/logger.js';

// Builds one `elements[]` entry (a group + its items) in the legacy shape,
// adding `itemId`/`groupId` on each item (additive — every field the client
// already reads is still there, in the same place).
function groupToLegacy(group) {
  const items = (group.items || []).map((item) => ({
    latitude: item.latitude,
    longitude: item.longitude,
    name: item.name,
    itemId: item.id,
    groupId: group.id,
  }));
  return {
    groupId: group.id,
    type: group.typeId,
    name: group.name,
    description: group.description,
    linea: group.linea,
    items,
  };
}

function baseToLegacy(base) {
  const legacy = { latitude: base.latitude, longitude: base.longitude, id: base.id };
  if (base.name) legacy.name = base.name;
  if (base.corners) legacy.corners = base.corners;
  return legacy;
}

function assignmentToLegacy(assignment) {
  return {
    baseId: assignment.baseId,
    device: assignment.device ? { id: String(assignment.device.id), name: assignment.device.name } : null,
    settings: assignment.settings,
  };
}

export const markersSnapshotModel = {
  // Legacy shape: { markersbase: [...], elements: [...] }
  async getMarkers() {
    const [groups, bases] = await Promise.all([
      sequelize.models.ElementGroup.findAll({
        where: { deletedAt: null },
        include: [{ model: sequelize.models.ElementItem, as: 'items' }],
      }),
      sequelize.models.Base.findAll(),
    ]);
    return {
      markersbase: bases.map(baseToLegacy),
      elements: groups.map(groupToLegacy),
    };
  },

  // Legacy shape: bases with their assigned device attached (markersModel.getBaseswithAssignments)
  async getBaseswithAssignments() {
    const [bases, assignments] = await Promise.all([
      sequelize.models.Base.findAll(),
      sequelize.models.Assignment.findAll({ include: [{ model: sequelize.models.Device, as: 'device' }] }),
    ]);
    const assignmentsMap = new Map(assignments.map((a) => [a.baseId, a.device]));
    return bases.map((base) => ({ ...baseToLegacy(base), device: assignmentsMap.get(base.id) || null }));
  },

  // Legacy shape: assignments[] as stored in the YAML ({baseId, device:{id,name}, settings})
  async getAssignments() {
    const assignments = await sequelize.models.Assignment.findAll({
      include: [{ model: sequelize.models.Device, as: 'device' }],
    });
    return assignments.map(assignmentToLegacy);
  },

  // Upserts markersbase/elements/assignments from a legacy-shaped payload
  // (same one the client already sends via POST /api/planning/setDefault).
  // Full-replace semantics per collection, matching today's "save everything
  // at once" behavior — not a real diff/patch.
  async setMarkers({ markersbase, elements, assignments }) {
    if (Array.isArray(markersbase)) {
      await this._upsertBases(markersbase);
    }
    if (Array.isArray(elements)) {
      await this._upsertElements(elements);
    }
    if (Array.isArray(assignments)) {
      await this._upsertAssignments(assignments);
    }
    return { result: true };
  },

  async _upsertBases(markersbase) {
    for (const base of markersbase) {
      if (!base.id) continue;
      await sequelize.models.Base.upsert({
        id: base.id,
        name: base.name ?? null,
        latitude: base.latitude,
        longitude: base.longitude,
        corners: base.corners ?? null,
      });
    }
  },

  // Matches existing groups by name+type (same heuristic as the migration
  // script) so re-saving the same YAML-shaped payload doesn't create
  // duplicates; items are matched by name within their group.
  async _upsertElements(elements) {
    for (const group of elements) {
      const name = (group.name || '').trim();
      const [elementGroup] = await sequelize.models.ElementGroup.findOrCreate({
        where: { name, typeId: group.type, deletedAt: null },
        defaults: {
          typeId: group.type,
          name,
          description: group.description || '',
          linea: group.linea ?? false,
          attributes: {},
        },
      });
      elementGroup.description = group.description || '';
      elementGroup.linea = group.linea ?? false;
      await elementGroup.save();

      for (const item of group.items || []) {
        const [elementItem] = await sequelize.models.ElementItem.findOrCreate({
          where: { groupId: elementGroup.id, name: item.name },
          defaults: {
            groupId: elementGroup.id,
            name: item.name,
            latitude: item.latitude,
            longitude: item.longitude,
          },
        });
        elementItem.latitude = item.latitude;
        elementItem.longitude = item.longitude;
        await elementItem.save();
      }
    }
  },

  async _upsertAssignments(assignments) {
    for (const assignment of assignments) {
      const deviceRef = assignment.device || {};
      const device = await sequelize.models.Device.findOne({
        where: deviceRef.name ? { name: deviceRef.name } : { id: deviceRef.id },
      });
      if (!device) {
        logger.warn(`setMarkers: skipping assignment for base ${assignment.baseId}, device not found`, {
          deviceRef,
        });
        continue;
      }
      const [record] = await sequelize.models.Assignment.findOrCreate({
        where: { baseId: assignment.baseId, deviceId: device.id },
        defaults: { baseId: assignment.baseId, deviceId: device.id, settings: assignment.settings || {} },
      });
      record.settings = assignment.settings || {};
      await record.save();
    }
  },
};
