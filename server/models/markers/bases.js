import sequelize from '../../common/sequelize.js';
import { warnUnknownParameterKeys, applyParameterDefaults } from './elementItems.js';
import { resolveEffectiveParameterDefs } from './elementTypes.js';

export const basesModel = {
  async getAll() {
    return await sequelize.models.Base.findAll();
  },

  async getById(id) {
    return await sequelize.models.Base.findOne({ where: { id } });
  },

  async create(base) {
    const { typeId, name, latitude, longitude, altitude, azimFront, attributes, corners } = base;

    const type = typeId ? await sequelize.models.ElementType.findByPk(typeId) : null;
    const parameterDefs = resolveEffectiveParameterDefs(type);
    if (attributes) {
      warnUnknownParameterKeys(attributes, parameterDefs);
    }

    return await sequelize.models.Base.create({
      typeId: typeId ?? null,
      name: name ?? null,
      latitude,
      longitude,
      altitude: altitude ?? null,
      azimFront: azimFront ?? null,
      attributes: applyParameterDefaults(attributes, parameterDefs),
      corners: corners ?? null,
    });
  },

  async update(id, base) {
    const { typeId, name, latitude, longitude, altitude, azimFront, attributes, corners } = base;
    const myBase = await sequelize.models.Base.findOne({ where: { id } });
    if (!myBase) {
      return null;
    }
    if (typeId !== undefined) myBase.typeId = typeId;
    if (name !== undefined) myBase.name = name;
    if (latitude !== undefined) myBase.latitude = latitude;
    if (longitude !== undefined) myBase.longitude = longitude;
    if (altitude !== undefined) myBase.altitude = altitude;
    if (azimFront !== undefined) myBase.azimFront = azimFront;
    if (corners !== undefined) myBase.corners = corners;
    if (attributes !== undefined) {
      if (myBase.typeId) {
        const type = await sequelize.models.ElementType.findByPk(myBase.typeId);
        warnUnknownParameterKeys(attributes, resolveEffectiveParameterDefs(type));
      }
      myBase.attributes = attributes;
    }
    await myBase.save();
    return myBase;
  },

  async delete(id) {
    return await sequelize.models.Base.destroy({ where: { id } });
  },
};
