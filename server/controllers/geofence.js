import { geofenceModel } from '../models/geofence.js';

export const geofenceController = {
  async getAll(req, res) {
    try {
      const geofences = await geofenceModel.getAll();
      res.json(geofences);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getById(req, res) {
    try {
      const { id } = req.params;
      const geofence = await geofenceModel.getById(id);
      if (!geofence) {
        return res.status(404).json({ error: 'Geocerca no encontrada' });
      }
      res.json(geofence);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async create(req, res) {
    try {
      const geofence = req.body;
      const result = await geofenceModel.create(geofence);
      res.status(201).json(result);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async update(req, res) {
    try {
      const { id } = req.params;
      const geofence = req.body;
      const result = await geofenceModel.update(id, geofence);
      if (result.affectedRows === 0) {
        return res.status(404).json({ error: 'Geocerca no encontrada' });
      }
      res.json(result);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async delete(req, res) {
    try {
      const { id } = req.params;
      const result = await geofenceModel.delete(id);
      if (result.affectedRows === 0) {
        return res.status(404).json({ error: 'Geocerca no encontrada' });
      }
      res.json({ message: 'Geocerca eliminada correctamente' });
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },
};
