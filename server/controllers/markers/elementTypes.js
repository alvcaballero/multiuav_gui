import multer from 'multer';
import path from 'path';
import { elementTypesModel } from '../../models/markers/elementTypes.js';
import { validateElementType, validatePartialElementType } from '../../schemas/zod/markers.js';
import { logger } from '../../common/logger.js';

const storage = multer.diskStorage({
  destination: (req, file, cb) => {
    const dir = elementTypesModel.ensureAssetDir(req.params.id);
    cb(null, dir);
  },
  filename: (req, file, cb) => {
    const ext = path.extname(file.originalname);
    const assetType = req.assetType; // set by route handler
    cb(null, `${assetType}${ext}`);
  },
});

export const upload = multer({ storage });

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
};
