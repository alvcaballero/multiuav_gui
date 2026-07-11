import multer from 'multer';
import path from 'path';
import { fileURLToPath } from 'url';
import { markersModel } from '../models/markers.js';
import { logger } from '../common/logger.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ASSETS_DIR = path.resolve(__dirname, '../data/element-types');

const storage = multer.diskStorage({
  destination: (req, file, cb) => {
    const dir = markersModel.ensureAssetDir(req.params.id);
    cb(null, dir);
  },
  filename: (req, file, cb) => {
    const ext = path.extname(file.originalname);
    const assetType = req.assetType; // set by route handler
    cb(null, `${assetType}${ext}`);
  },
});

export const upload = multer({ storage });

export class markersController {
  // ─── Instances ────────────────────────────────────────────────────────────

  static async getMarkers(req, res) {
    const response = markersModel.getMarkers();
    res.json(response);
  }

  static async setMarkers(req, res) {
    const response = markersModel.setMarkers(req.body);
    res.json(response);
  }

  static async getBases(req, res) {
    const response = markersModel.getBases();
    res.json(response);
  }

  static async getElements(req, res) {
    const response = markersModel.getElements();
    res.json(response);
  }

  static async getBasesWithAssignments(req, res) {
    const response = markersModel.getBaseswithAssignments();
    res.json(response);
  }

  // ─── Type catalog ─────────────────────────────────────────────────────────

  static async getAllTypes(req, res) {
    const response = markersModel.getAllTypes();
    res.json(response);
  }

  static async createCustomType(req, res) {
    try {
      const response = markersModel.createCustomType(req.body);
      res.status(201).json(response);
    } catch (err) {
      logger.error('Error creating custom type', { error: err.message });
      res.status(400).json({ error: err.message });
    }
  }

  static async deleteCustomType(req, res) {
    try {
      const response = markersModel.deleteCustomType(req.params.id);
      res.json(response);
    } catch (err) {
      logger.error('Error deleting custom type', { error: err.message });
      res.status(404).json({ error: err.message });
    }
  }

  static async uploadIcon(req, res) {
    try {
      if (!req.file) return res.status(400).json({ error: 'No file uploaded' });
      const response = markersModel.saveTypeIcon(req.params.id, req.file.path);
      res.json(response);
    } catch (err) {
      logger.error('Error uploading icon', { error: err.message });
      res.status(404).json({ error: err.message });
    }
  }

  static async uploadModel(req, res) {
    try {
      if (!req.file) return res.status(400).json({ error: 'No file uploaded' });
      const response = markersModel.saveTypeModel(req.params.id, req.file.path);
      res.json(response);
    } catch (err) {
      logger.error('Error uploading model', { error: err.message });
      res.status(404).json({ error: err.message });
    }
  }

  static async serveIcon(req, res) {
    const filePath = markersModel.getAssetPath(req.params.id, 'icon');
    if (!filePath) return res.status(404).json({ error: 'Icon not found' });
    res.sendFile(filePath);
  }

  static async serveModel(req, res) {
    const filePath = markersModel.getAssetPath(req.params.id, 'model');
    if (!filePath) return res.status(404).json({ error: 'Model not found' });
    res.sendFile(filePath);
  }
}
