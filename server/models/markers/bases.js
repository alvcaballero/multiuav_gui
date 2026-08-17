import sequelize from '../../common/sequelize.js';

export const basesModel = {
  async getAll() {
    return await sequelize.models.Base.findAll();
  },

  async getById(id) {
    return await sequelize.models.Base.findOne({ where: { id } });
  },

  async create(base) {
    const { typeId, name, latitude, longitude, corners } = base;

    return await sequelize.models.Base.create({
      typeId: typeId ?? null,
      name: name ?? null,
      latitude,
      longitude,
      corners: corners ?? null,
    });
  },

  async update(id, base) {
    const { typeId, name, latitude, longitude, corners } = base;
    const myBase = await sequelize.models.Base.findOne({ where: { id } });
    if (!myBase) {
      return null;
    }
    if (typeId !== undefined) myBase.typeId = typeId;
    if (name !== undefined) myBase.name = name;
    if (latitude !== undefined) myBase.latitude = latitude;
    if (longitude !== undefined) myBase.longitude = longitude;
    if (corners !== undefined) myBase.corners = corners;
    await myBase.save();
    return myBase;
  },

  async delete(id) {
    return await sequelize.models.Base.destroy({ where: { id } });
  },
};
