import { Op } from 'sequelize';
import sequelize from '../../common/sequelize.js';
import { elementGroupsModel } from './elementGroups.js';

/**
 * Copies `defaultGeometry` (from the item's ElementType) into `attributes.geometry`
 * when the item doesn't already carry its own geometry. Pure/no I/O so it's
 * testable in isolation; other attribute keys are preserved either way.
 */
export function mergeDefaultGeometry(attributes, defaultGeometry) {
  if (attributes?.geometry || !defaultGeometry) return attributes ?? null;
  return { ...(attributes ?? {}), geometry: defaultGeometry };
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
    const { groupId, name, latitude, longitude, description, attributes } = elementItem;

    let finalAttributes = attributes ?? null;
    if (!finalAttributes?.geometry) {
      const group = await sequelize.models.ElementGroup.findByPk(groupId, {
        include: [{ model: sequelize.models.ElementType, as: 'type' }],
      });
      finalAttributes = mergeDefaultGeometry(finalAttributes, group?.type?.attributes?.geometry);
    }

    const created = await sequelize.models.ElementItem.create({
      groupId,
      name,
      latitude,
      longitude,
      description: description ?? null,
      attributes: finalAttributes,
    });
    await elementGroupsModel.recalculateBounds(groupId);
    return created;
  },

  async update(id, elementItem) {
    const { groupId, name, latitude, longitude, description, attributes } = elementItem;
    const myItem = await sequelize.models.ElementItem.findOne({ where: { id } });
    if (!myItem) {
      return null;
    }
    const previousGroupId = myItem.groupId;
    if (groupId) myItem.groupId = groupId;
    if (name) myItem.name = name;
    if (latitude !== undefined) myItem.latitude = latitude;
    if (longitude !== undefined) myItem.longitude = longitude;
    if (description !== undefined) myItem.description = description;
    if (attributes !== undefined) myItem.attributes = attributes;
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
