import sequelize from '../../common/sequelize.js';

export const elementItemsModel = {
  async getAll() {
    return await sequelize.models.ElementItem.findAll();
  },

  async getById(id) {
    return await sequelize.models.ElementItem.findOne({ where: { id } });
  },

  async findByGroup(groupId) {
    return await sequelize.models.ElementItem.findAll({ where: { groupId } });
  },

  async create(elementItem) {
    const { groupId, name, latitude, longitude, description, attributes } = elementItem;

    return await sequelize.models.ElementItem.create({
      groupId,
      name,
      latitude,
      longitude,
      description: description ?? null,
      attributes: attributes ?? null,
    });
  },

  async update(id, elementItem) {
    const { groupId, name, latitude, longitude, description, attributes } = elementItem;
    const myItem = await sequelize.models.ElementItem.findOne({ where: { id } });
    if (!myItem) {
      return null;
    }
    if (groupId) myItem.groupId = groupId;
    if (name) myItem.name = name;
    if (latitude !== undefined) myItem.latitude = latitude;
    if (longitude !== undefined) myItem.longitude = longitude;
    if (description !== undefined) myItem.description = description;
    if (attributes !== undefined) myItem.attributes = attributes;
    await myItem.save();
    return myItem;
  },

  async delete(id) {
    return await sequelize.models.ElementItem.destroy({ where: { id } });
  },
};
