import { Op } from 'sequelize';
import sequelize from '../../common/sequelize.js';
import { logger } from '../../common/logger.js';
import { elementGroupsModel } from './elementGroups.js';
import { resolveEffectiveParameterDefs } from './elementTypes.js';

/**
 * Warns (never rejects) when an item's attribute keys aren't declared on its
 * ElementType's `parameterDefs`. Soft check on purpose: unlike `geometry`
 * (an invariant — same type, same dimensions, enforced at the zod layer),
 * an unknown parameter key isn't a violation, just a heads-up — and this
 * path's sibling, `_upsertElements`, doesn't validate against the type's
 * schema at parse time either.
 */
export function warnUnknownParameterKeys(attributes, parameterDefs) {
  if (!attributes) return;
  const knownKeys = new Set((parameterDefs || []).map((def) => def.key));
  for (const key of Object.keys(attributes)) {
    if (!knownKeys.has(key)) {
      logger.warn(`ElementItem attributes: key "${key}" is not defined in its ElementType's parameterDefs`);
    }
  }
}

/**
 * Merges an ElementType's parameterDefs.default values into `attributes` for
 * any key not already present. Used ONLY at creation time — an item/base's
 * `attributes` must be a real, persisted snapshot of its parameter values,
 * never relying on a client form silently falling back to the type's current
 * default for display (that let the DB and what the UI showed diverge: a
 * form could show `nacelle_heading_deg: 240` from the type's default while
 * the item's own `attributes` stayed `null`, and a GET would return nothing).
 * A def with no `default` contributes no key. Not applied on update — an
 * edit is a deliberate full-replace of `attributes`, not a place to
 * resurrect defaults for keys the form omitted.
 */
export function applyParameterDefaults(attributes, parameterDefs) {
  if (!parameterDefs || parameterDefs.length === 0) return attributes ?? null;
  const merged = { ...(attributes || {}) };
  for (const def of parameterDefs) {
    if (def.default === undefined) continue;
    if (!(def.key in merged)) merged[def.key] = def.default;
  }
  return Object.keys(merged).length > 0 ? merged : null;
}

// Excludes items whose parent group is soft-deleted — a deleted group keeps
// its items in SQL (intentional, see elementGroupsModel.delete), but nothing
// reading items for external use (REST, MCP tools, mission planning) should
// ever see them. `required: true` makes this an inner join, so a mismatched/
// deleted group filters the item out instead of returning it with `group:
// null`.
function groupFilter(sequelize) {
  return {
    model: sequelize.models.ElementGroup,
    as: 'group',
    attributes: [],
    where: { deletedAt: null },
    required: true,
  };
}

export const elementItemsModel = {
  async getAll() {
    return await sequelize.models.ElementItem.findAll({ include: [groupFilter(sequelize)] });
  },

  async getById(id) {
    return await sequelize.models.ElementItem.findOne({ where: { id }, include: [groupFilter(sequelize)] });
  },

  async getDetailById(id) {
    return await sequelize.models.ElementItem.findOne({
      where: { id },
      include: [
        {
          model: sequelize.models.ElementGroup,
          as: 'group',
          where: { deletedAt: null },
          required: true,
          include: [{ model: sequelize.models.ElementType, as: 'type' }],
        },
      ],
    });
  },

  async getByIds(ids) {
    return await sequelize.models.ElementItem.findAll({
      where: { id: { [Op.in]: ids } },
      include: [groupFilter(sequelize)],
    });
  },

  /**
   * ElementItems whose lat/lng falls inside a geographic bounding box.
   * @param {{minLat:number, maxLat:number, minLng:number, maxLng:number}} bounds
   */
  async getAllInBounds({ minLat, maxLat, minLng, maxLng }) {
    return await sequelize.models.ElementItem.findAll({
      where: {
        latitude: { [Op.between]: [minLat, maxLat] },
        longitude: { [Op.between]: [minLng, maxLng] },
      },
      include: [groupFilter(sequelize)],
    });
  },

  async findByGroup(groupId) {
    return await sequelize.models.ElementItem.findAll({
      where: { groupId },
      include: [groupFilter(sequelize)],
    });
  },

  async create(elementItem) {
    const { groupId, name, latitude, longitude, altitude, azimFront, description, attributes } = elementItem;

    const group = await sequelize.models.ElementGroup.findByPk(groupId, {
      include: [{ model: sequelize.models.ElementType, as: 'type' }],
    });
    const parameterDefs = resolveEffectiveParameterDefs(group?.type);
    if (attributes) {
      warnUnknownParameterKeys(attributes, parameterDefs);
    }

    const created = await sequelize.models.ElementItem.create({
      groupId,
      name,
      latitude,
      longitude,
      altitude: altitude ?? null,
      azimFront: azimFront ?? null,
      description: description ?? null,
      attributes: applyParameterDefaults(attributes, parameterDefs),
    });
    await elementGroupsModel.recalculateBounds(groupId);
    return created;
  },

  async update(id, elementItem) {
    const { groupId, name, latitude, longitude, altitude, azimFront, description, attributes } = elementItem;
    const myItem = await sequelize.models.ElementItem.findOne({ where: { id } });
    if (!myItem) {
      return null;
    }
    const previousGroupId = myItem.groupId;
    if (groupId) myItem.groupId = groupId;
    if (name) myItem.name = name;
    if (latitude !== undefined) myItem.latitude = latitude;
    if (longitude !== undefined) myItem.longitude = longitude;
    if (altitude !== undefined) myItem.altitude = altitude;
    if (azimFront !== undefined) myItem.azimFront = azimFront;
    if (description !== undefined) myItem.description = description;
    if (attributes !== undefined) {
      const group = await sequelize.models.ElementGroup.findByPk(myItem.groupId, {
        include: [{ model: sequelize.models.ElementType, as: 'type' }],
      });
      warnUnknownParameterKeys(attributes, resolveEffectiveParameterDefs(group?.type));
      myItem.attributes = attributes;
    }
    await myItem.save();

    await elementGroupsModel.recalculateBounds(myItem.groupId);
    if (groupId && groupId !== previousGroupId) {
      await elementGroupsModel.recalculateBounds(previousGroupId);
    }
    return myItem;
  },

  async delete(id) {
    const myItem = await sequelize.models.ElementItem.findOne({ where: { id } });
    const result = await sequelize.models.ElementItem.destroy({ where: { id } });
    if (myItem) {
      await elementGroupsModel.recalculateBounds(myItem.groupId);
    }
    return result;
  },
};
