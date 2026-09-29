import multer from 'multer';
import path from 'path';
import fs from 'fs';
import { parse } from 'yaml';
import { elementTypesModel } from '../../models/markers/elementTypes.js';
import { resolveDefinitionModel, DefinitionResolveError } from '../../models/markers/definitionResolver.js';
import { validateElementType, validatePartialElementType } from '../../schemas/zod/markers.js';
import { logger } from '../../common/logger.js';

const storage = multer.diskStorage({
  destination: async (req, file, cb) => {
    const dir = await elementTypesModel.ensureAssetDir(req.params.id);
    cb(null, dir);
  },
  filename: (req, file, cb) => {
    const ext = path.extname(file.originalname);
    const assetType = req.assetType; // set by route handler
    cb(null, `${assetType}${ext}`);
  },
});

export const upload = multer({ storage });

// The definition upload needs to validate content BEFORE it touches disk —
// `upload` (diskStorage) writes the file as a side effect of the multer
// middleware itself, ahead of any route-handler code, so validating after
// that would mean the bad upload already clobbered a previously good file.
// Memory storage keeps the buffer in `req.file.buffer` until the controller
// explicitly writes it.
export const uploadDefinitionFile = multer({ storage: multer.memoryStorage() });

export function setIconAssetType(req, res, next) {
  req.assetType = 'icon';
  next();
}

export function setModelAssetType(req, res, next) {
  req.assetType = 'model';
  next();
}

export const elementTypesController = {
  async getAll(req, res) {
    try {
      const elementTypes = await elementTypesModel.getAll();
      res.json(elementTypes);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getById(req, res) {
    try {
      const { id } = req.params;
      const elementType = await elementTypesModel.getById(id);
      if (!elementType) {
        return res.status(404).json({ error: 'Tipo de elemento no encontrado' });
      }
      res.json(elementType);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async create(req, res) {
    try {
      const result = validateElementType(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const elementType = await elementTypesModel.create(result.data);
      res.status(201).json(elementType);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async update(req, res) {
    try {
      const { id } = req.params;
      const result = validatePartialElementType(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const elementType = await elementTypesModel.update(id, result.data);
      if (!elementType) {
        return res.status(404).json({ error: 'Tipo de elemento no encontrado' });
      }
      res.json(elementType);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async delete(req, res) {
    try {
      const { id } = req.params;
      const affected = await elementTypesModel.delete(id);
      if (affected === 0) {
        return res.status(404).json({ error: 'Tipo de elemento no encontrado' });
      }
      res.json({ message: 'Tipo de elemento eliminado correctamente' });
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  // ─── Assets (icon/model) ──────────────────────────────────────────────────

  async serveIcon(req, res) {
    const filePath = elementTypesModel.getAssetPath(req.params.id, 'icon');
    if (!filePath) return res.status(404).json({ error: 'Icon not found' });
    res.sendFile(filePath);
  },

  async serveModel(req, res) {
    const filePath = elementTypesModel.getAssetPath(req.params.id, 'model');
    if (!filePath) return res.status(404).json({ error: 'Model not found' });
    res.sendFile(filePath);
  },

  async serveDefinition(req, res) {
    const filePath = elementTypesModel.getAssetPath(req.params.id, 'definition');
    if (!filePath) return res.status(404).json({ error: 'Definition not found' });
    res.type('yaml').sendFile(filePath);
  },

  // Resolves the definition file into a plain link tree (expressions already
  // evaluated) for the "Comprobar YAML" 3D comparison viewer. `content` in
  // the body, when present, is evaluated directly instead of the file on
  // disk — an unsaved edit in the client's YAML editor doesn't need to be
  // saved first to be checked.
  async resolveDefinition(req, res) {
    try {
      const { id } = req.params;
      const content = typeof req.body?.content === 'string' ? req.body.content : undefined;
      const model = resolveDefinitionModel(id, content);
      if (!model) return res.status(404).json({ error: 'No definition file for this type' });
      res.json(model);
    } catch (error) {
      if (error instanceof DefinitionResolveError) {
        return res.status(400).json({ error: error.message, link: error.link, expression: error.expression });
      }
      logger.error('Error resolving definition model', { error: error.message });
      res.status(500).json({ error: error.message });
    }
  },

  async uploadIcon(req, res) {
    try {
      if (!req.file) return res.status(400).json({ error: 'No file uploaded' });
      const { id } = req.params;
      const elementType = await elementTypesModel.update(id, { icon: `/api/markers/types/${id}/icon` });
      if (!elementType) return res.status(404).json({ error: 'Tipo de elemento no encontrado' });
      res.json(elementType);
    } catch (error) {
      logger.error('Error uploading icon', { error: error.message });
      res.status(500).json({ error: error.message });
    }
  },

  async uploadModel(req, res) {
    try {
      if (!req.file) return res.status(400).json({ error: 'No file uploaded' });
      const { id } = req.params;
      const elementType = await elementTypesModel.update(id, { model3d: `/api/markers/types/${id}/model` });
      if (!elementType) return res.status(404).json({ error: 'Tipo de elemento no encontrado' });
      res.json(elementType);
    } catch (error) {
      logger.error('Error uploading model', { error: error.message });
      res.status(500).json({ error: error.message });
    }
  },

  async uploadDefinition(req, res) {
    try {
      if (!req.file) return res.status(400).json({ error: 'No file uploaded' });
      const content = req.file.buffer.toString('utf8');
      // Validate BEFORE writing anything — a rejected upload must leave any
      // previously stored definition untouched (see uploadDefinitionFile).
      try {
        parse(content);
      } catch (parseError) {
        return res.status(400).json({ error: `Invalid YAML: ${parseError.message}` });
      }
      const { id } = req.params;
      const dir = await elementTypesModel.ensureAssetDir(id);
      // Always stored as .yaml — clear a stale .yml sibling so getAssetPath
      // can't pick up an older extension after re-uploading with a new one.
      fs.rmSync(path.join(dir, 'definition.yml'), { force: true });
      fs.writeFileSync(path.join(dir, 'definition.yaml'), content);
      const elementType = await elementTypesModel.update(id, {
        definitionYaml: `/api/markers/types/${id}/definition`,
      });
      if (!elementType) return res.status(404).json({ error: 'Tipo de elemento no encontrado' });
      res.json(elementType);
    } catch (error) {
      logger.error('Error uploading definition', { error: error.message });
      res.status(500).json({ error: error.message });
    }
  },
};
