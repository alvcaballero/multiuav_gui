import path from 'path';
import fs from 'fs';
import { fileURLToPath } from 'url';
import sequelize from '../../common/sequelize.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ASSETS_DIR = path.resolve(__dirname, '../../data/element-types');

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

  // ─── Assets (icon/model files) ────────────────────────────────────────────
  // Filesystem storage, not SQL — kept here (not in a separate module)
  // because it's small and only ever used alongside the type catalog CRUD.

  getAssetPath(id, assetType) {
    const ext = assetType === 'icon' ? ['png', 'svg', 'jpg'] : ['glb', 'gltf'];
    const dir = path.join(ASSETS_DIR, id);
    if (!fs.existsSync(dir)) return null;

    for (const e of ext) {
      const p = path.join(dir, `${assetType}.${e}`);
      if (fs.existsSync(p)) return p;
    }
    return null;
  },

  ensureAssetDir(id) {
    const dir = path.join(ASSETS_DIR, id);
    if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
    return dir;
  },
};
