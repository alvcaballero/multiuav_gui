import sequelize from '../../common/sequelize.js';

export const elementTypesModel = {
  async getAll() {
    return await sequelize.models.ElementType.findAll();
  },

  async getById(id) {
    return await sequelize.models.ElementType.findOne({ where: { id } });
  },

  async create(elementType) {
    const { id, name, description, icon, model3d, color, isCustom } = elementType;

    return await sequelize.models.ElementType.create({
      id,
      name,
      description: description || '',
      icon: icon ?? null,
      model3d: model3d ?? null,
      color: color ?? null,
      isCustom: isCustom ?? false,
    });
  },

  async update(id, elementType) {
    const { name, description, icon, model3d, color, isCustom } = elementType;
    const myType = await sequelize.models.ElementType.findOne({ where: { id } });
    if (!myType) {
      return null;
    }
    if (name) myType.name = name;
    if (description !== undefined) myType.description = description;
    if (icon !== undefined) myType.icon = icon;
    if (model3d !== undefined) myType.model3d = model3d;
    if (color !== undefined) myType.color = color;
    if (isCustom !== undefined) myType.isCustom = isCustom;
    await myType.save();
    return myType;
  },

  async delete(id) {
    return await sequelize.models.ElementType.destroy({ where: { id } });
  },
};
