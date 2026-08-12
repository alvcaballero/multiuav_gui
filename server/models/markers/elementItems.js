import { Op } from 'sequelize';
import sequelize from '../../common/sequelize.js';

export const elementItemsModel = {
  async getAll() {
    return await sequelize.models.ElementItem.findAll();
  },

  async getById(id) {
    return await sequelize.models.ElementItem.findOne({ where: { id } });
  },

  async getDetailById(id) {
    return await sequelize.models.ElementItem.findOne({
      where: { id },
      include: [
        {
          model: sequelize.models.ElementGroup,
          as: 'group',
          include: [{ model: sequelize.models.ElementType, as: 'type' }],
        },
      ],
    });
  },

  async getByIds(ids) {
    return await sequelize.models.ElementItem.findAll({ where: { id: { [Op.in]: ids } } });
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
    });
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
