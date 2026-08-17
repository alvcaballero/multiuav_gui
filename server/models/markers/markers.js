// Marker instances (bases + elements + assignments), backed by SQL
// (ElementGroup/ElementItem/Base/Assignment). Exposes the legacy YAML-shaped
// `{markersbase, elements}` blob — still what the client/planner speak — the
// wire format doesn't change, only where the data actually lives.
import { Op } from 'sequelize';
import sequelize from '../../common/sequelize.js';
import { logger } from '../../common/logger.js';
import { elementGroupsModel } from './elementGroups.js';

// Builds one `elements[]` entry (a group + its items) in the legacy shape,
// adding `itemId`/`groupId`/`attributes` on each item and `attributes` on the
// group (additive — every field the client already reads is still there, in
// the same place). `attributes` is what lets get_registered_objects
// (mcp_server) hand the LLM structured data instead of parsing `description`
// free text.
function groupToLegacy(group) {
  const items = (group.items || []).map((item) => ({
    latitude: item.latitude,
    longitude: item.longitude,
    name: item.name,
    itemId: item.id,
    groupId: group.id,
    attributes: item.attributes,
  }));
  return {
    groupId: group.id,
    type: group.typeId,
    name: group.name,
    description: group.description,
    linea: group.linea,
    attributes: group.attributes,
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

export const markersModel = {
  // ─── Marker instances (bases + elements) ─────────────────────────────────

  // Legacy shape: { markersbase: [...], elements: [...] }
  // `groupIds`, when given, restricts `elements` to those groups only —
  // used by get_registered_objects (mcp_server) to avoid dumping every
  // registered group into the LLM's context at once.
  async getMarkers({ groupIds } = {}) {
    const groupWhere = { deletedAt: null };
    if (Array.isArray(groupIds) && groupIds.length > 0) {
      groupWhere.id = { [Op.in]: groupIds };
    }
    const [groups, bases] = await Promise.all([
      sequelize.models.ElementGroup.findAll({
        where: groupWhere,
        include: [{ model: sequelize.models.ElementItem, as: 'items' }],
      }),
      sequelize.models.Base.findAll(),
    ]);
    return {
      markersbase: bases.map(baseToLegacy),
      elements: groups.map(groupToLegacy),
    };
  },

  // Upserts markersbase/elements/assignments from a legacy-shaped payload
  // (same one the client already sends via POST /api/planning/setDefault).
  // Full-replace semantics per collection: a group/item/base present in SQL
  // but missing from the incoming payload is removed, matching what "Save
  // Global Markers" is supposed to mean in the UI. Returns the fully-synced
  // state read back from SQL (including any id the server just assigned to
  // a newly-created base) so the caller can hand it back to the client.
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
    return await this.getMarkers();
  },

  // A base is only updated in place when its `id` matches a row that
  // actually exists in the table — never a blind upsert by whatever id the
  // client happened to send. A base with no id, or an id that doesn't match
  // any existing row (a client-side placeholder, a stale/deleted id), is
  // always inserted as a new row and gets a real id from AUTOINCREMENT. This
  // is what makes two different client-generated placeholder ids collide
  // safely instead of overwriting each other's data.
  async _upsertBases(markersbase) {
    const existing = await sequelize.models.Base.findAll();
    const byId = new Map(existing.map((row) => [row.id, row]));
    for (const base of markersbase) {
      const numericId = base.id != null ? Number(base.id) : null;
      const row = numericId != null && !Number.isNaN(numericId) ? byId.get(numericId) : null;

      if (row) {
        row.name = base.name ?? null;
        row.latitude = base.latitude;
        row.longitude = base.longitude;
        row.corners = base.corners ?? null;
        if (base.typeId !== undefined) row.typeId = base.typeId;
        await row.save();
      } else {
        await sequelize.models.Base.create({
          typeId: base.typeId ?? null,
          name: base.name ?? null,
          latitude: base.latitude,
          longitude: base.longitude,
          corners: base.corners ?? null,
        });
      }
    }
  },

  // Matches existing groups by id when the payload has one (every group the
  // server ever sent out carries its real `groupId`, per groupToLegacy
  // above); falls back to name+type for a group created client-side and
  // never saved yet. Items are matched the same way by `itemId`/name within
  // their group. A group/item that exists in SQL but is missing from the
  // payload was deleted client-side — it's soft-deleted (groups) / hard-deleted
  // (items), same as the fine-grained DELETE endpoints already do, so "Save
  // Global Markers" is a real full-replace instead of insert/update-only.
  async _upsertElements(elements) {
    const existingGroups = await sequelize.models.ElementGroup.findAll({ where: { deletedAt: null } });
    const seenGroupIds = new Set();

    for (const group of elements) {
      const name = (group.name || '').trim();
      const groupId = group.groupId != null ? Number(group.groupId) : null;
      let elementGroup =
        groupId != null && !Number.isNaN(groupId)
          ? existingGroups.find((g) => g.id === groupId)
          : null;

      if (!elementGroup) {
        [elementGroup] = await sequelize.models.ElementGroup.findOrCreate({
          where: { name, typeId: group.type, deletedAt: null },
          defaults: {
            typeId: group.type,
            name,
            description: group.description || '',
            linea: group.linea ?? false,
            attributes: {},
          },
        });
      }
      seenGroupIds.add(elementGroup.id);

      elementGroup.name = name;
      elementGroup.description = group.description || '';
      elementGroup.linea = group.linea ?? false;
      await elementGroup.save();

      const existingItems = await sequelize.models.ElementItem.findAll({ where: { groupId: elementGroup.id } });
      const seenItemIds = new Set();

      const items = group.items || [];
      for (let i = 0; i < items.length; i++) {
        const item = items[i];
        // The client lets a new item be created without a `name` (the field
        // is optional in the UI) — Sequelize's `where` rejects `undefined`
        // outright, which was silently aborting the rest of this loop (an
        // unhandled rejection), leaving the group saved but every one of its
        // items unsaved. Fall back to a positional placeholder so the lookup
        // is always well-formed; the client's own placeholder for a nameless
        // row follows the same "Type index" pattern (see BaseList.jsx).
        const name = item.name || `Item ${i}`;
        const itemId = item.itemId != null ? Number(item.itemId) : null;
        let elementItem =
          itemId != null && !Number.isNaN(itemId) ? existingItems.find((it) => it.id === itemId) : null;

        if (!elementItem) {
          [elementItem] = await sequelize.models.ElementItem.findOrCreate({
            where: { groupId: elementGroup.id, name },
            defaults: {
              groupId: elementGroup.id,
              name,
              latitude: item.latitude,
              longitude: item.longitude,
            },
          });
        }
        seenItemIds.add(elementItem.id);

        elementItem.name = name;
        elementItem.latitude = item.latitude;
        elementItem.longitude = item.longitude;
        await elementItem.save();
      }

      const removedItemIds = existingItems.filter((it) => !seenItemIds.has(it.id)).map((it) => it.id);
      if (removedItemIds.length > 0) {
        await sequelize.models.ElementItem.destroy({ where: { id: { [Op.in]: removedItemIds } } });
      }

      await elementGroupsModel.recalculateBounds(elementGroup.id);
    }

    const removedGroupIds = existingGroups.filter((g) => !seenGroupIds.has(g.id)).map((g) => g.id);
    if (removedGroupIds.length > 0) {
      await sequelize.models.ElementGroup.update(
        { deletedAt: new Date() },
        { where: { id: { [Op.in]: removedGroupIds }, deletedAt: null } }
      );
    }
  },

  async _upsertAssignments(assignments) {
    for (const assignment of assignments) {
      const baseId = Number(assignment.baseId);
      const deviceRef = assignment.device || {};
      const device = await sequelize.models.Device.findOne({
        where: deviceRef.name ? { name: deviceRef.name } : { id: deviceRef.id },
      });
      if (!device) {
        logger.warn(`setMarkers: skipping assignment for base ${baseId}, device not found`, {
          deviceRef,
        });
        continue;
      }
      const [record] = await sequelize.models.Assignment.findOrCreate({
        where: { baseId, deviceId: device.id },
        defaults: { baseId, deviceId: device.id, settings: assignment.settings || {} },
      });
      record.settings = assignment.settings || {};
      await record.save();
    }
  },

  // Legacy shape: bases with their assigned device attached
  async getBaseswithAssignments() {
    const [bases, assignments] = await Promise.all([
      sequelize.models.Base.findAll(),
      sequelize.models.Assignment.findAll({ include: [{ model: sequelize.models.Device, as: 'device' }] }),
    ]);
    const assignmentsMap = new Map(assignments.map((a) => [Number(a.baseId), a.device]));
    return bases.map((base) => ({ ...baseToLegacy(base), device: assignmentsMap.get(Number(base.id)) || null }));
  },

  // Legacy shape: assignments[] as stored in the YAML ({baseId, device:{id,name}, settings})
  async getAssignments() {
    const assignments = await sequelize.models.Assignment.findAll({
      include: [{ model: sequelize.models.Device, as: 'device' }],
    });
    return assignments.map(assignmentToLegacy);
  },
};
