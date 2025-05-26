import sequelize from '../common/sequelize.js';

export const geofenceModel = {
  async getAll() {
    return await sequelize.models.Geofence.findAll();
  },

  async getById(id) {
    return await sequelize.models.Geofence.findOne({ where: { id: id } });
  },

  async create(geofence) {
    const { attributes, name, description, area } = geofence;

    return await await sequelize.models.Geofence.create({ attributes: {}, name, description: '', area });
  },

  async update(id, geofence) {
    const { attributes, name, description, area } = geofence;
    let myGeofence = await sequelize.models.Geofence.findOne({ where: { id: id } });
    if (!myGeofence) {
      return null;
    }
    if (name) myGeofence.name = name;
    if (area) myGeofence.area = area;
    if (description) myGeofence.description = description;
    if (attributes) myGeofence.attributes = attributes;
    await myGeofence.save();
    return myGeofence;
  },

  async delete(id) {
    return await sequelize.models.Geofence.destroy({ where: { id: id } });
  },
};
