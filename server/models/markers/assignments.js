import sequelize from '../../common/sequelize.js';

export const assignmentsModel = {
  async getAll() {
    return await sequelize.models.Assignment.findAll();
  },

  async getById(id) {
    return await sequelize.models.Assignment.findOne({ where: { id } });
  },

  async create(assignment) {
    const { baseId, deviceId, settings } = assignment;

    return await sequelize.models.Assignment.create({
      baseId,
      deviceId,
      settings: settings || {},
    });
  },

  async update(id, assignment) {
    const { baseId, deviceId, settings } = assignment;
    const myAssignment = await sequelize.models.Assignment.findOne({ where: { id } });
    if (!myAssignment) {
      return null;
    }
    if (baseId) myAssignment.baseId = baseId;
    if (deviceId) myAssignment.deviceId = deviceId;
    if (settings !== undefined) myAssignment.settings = settings;
    await myAssignment.save();
    return myAssignment;
  },

  async delete(id) {
    return await sequelize.models.Assignment.destroy({ where: { id } });
  },

  // Replaces markersModel.getBaseswithAssignments(): every base, with its
  // assigned device (or null) attached.
  async getBaseswithAssignments() {
    const [bases, assignments] = await Promise.all([
      sequelize.models.Base.findAll(),
      sequelize.models.Assignment.findAll({ include: [{ model: sequelize.models.Device, as: 'device' }] }),
    ]);
    const assignmentsMap = new Map(assignments.map((a) => [a.baseId, a.device]));
    return bases.map((base) => ({ ...base.dataValues, device: assignmentsMap.get(base.id) || null }));
  },

  // Replaces planningModel.getBasesSettings(): consumed by
  // server/models/mission/mission.js decodeTask() — keep the {devices, settings,
  // base} shape exactly as before, since the field is called `devices` (not
  // `device`) on the legacy side despite being a single object.
  async getBasesSettings() {
    const assignments = await sequelize.models.Assignment.findAll({
      include: [
        { model: sequelize.models.Device, as: 'device' },
        { model: sequelize.models.Base, as: 'base' },
      ],
    });
    return assignments.map((a) => ({
      devices: a.device,
      settings: a.settings,
      base: a.base,
    }));
  },
};
