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
};
