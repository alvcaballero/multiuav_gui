import sequelize from '../../common/sequelize.js';

export const elementGroupsModel = {
  async getAll() {
    return await sequelize.models.ElementGroup.findAll({ where: { deletedAt: null } });
  },

  async getById(id) {
    return await sequelize.models.ElementGroup.findOne({ where: { id, deletedAt: null } });
  },

  async create(elementGroup) {
    const { typeId, name, description, linea, attributes } = elementGroup;

    return await sequelize.models.ElementGroup.create({
      typeId,
      name,
      description: description || '',
      linea: linea ?? false,
      attributes: attributes || {},
    });
  },

  async update(id, elementGroup) {
    const { typeId, name, description, linea, attributes } = elementGroup;
    const myGroup = await sequelize.models.ElementGroup.findOne({ where: { id, deletedAt: null } });
    if (!myGroup) {
      return null;
    }
    if (typeId) myGroup.typeId = typeId;
    if (name) myGroup.name = name;
    if (description !== undefined) myGroup.description = description;
    if (linea !== undefined) myGroup.linea = linea;
    if (attributes !== undefined) myGroup.attributes = attributes;
    await myGroup.save();
    return myGroup;
  },

  // Soft-delete: same pattern as Device (server/models/devices.js) — hides the
  // group from every getAll/getById query without losing its data or its items.
  async delete(id) {
    return await sequelize.models.ElementGroup.update(
      { deletedAt: new Date() },
      { where: { id, deletedAt: null } }
    );
  },

  // Recomputes the group's lat/lng bounding box from its current items and
  // persists it under attributes.bounds — the two corner points (min, max)
  // covering every item in the group. Merged into the existing `attributes`
  // JSON blob (same convention as mergeDefaultGeometry in elementItems.js:
  // attributes is a free-form bag, other keys are preserved). Called after
  // any write that adds/edits/removes an ElementItem. `null` when the group
  // has no items.
  async recalculateBounds(groupId) {
    const group = await sequelize.models.ElementGroup.findByPk(groupId);
    if (!group) return null;

    const items = await sequelize.models.ElementItem.findAll({
      where: { groupId },
      attributes: ['latitude', 'longitude'],
      raw: true,
    });

    const bounds = items.length
      ? items.reduce(
          (acc, { latitude, longitude }) => ({
            minLat: Math.min(acc.minLat, latitude),
            maxLat: Math.max(acc.maxLat, latitude),
            minLng: Math.min(acc.minLng, longitude),
            maxLng: Math.max(acc.maxLng, longitude),
          }),
          { minLat: items[0].latitude, maxLat: items[0].latitude, minLng: items[0].longitude, maxLng: items[0].longitude }
        )
      : null;

    group.attributes = { ...(group.attributes ?? {}), bounds };
    await group.save();
    return bounds;
  },
};
