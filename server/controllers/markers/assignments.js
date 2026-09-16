import { assignmentsModel } from '../../models/markers/assignments.js';
import { validateAssignment, validatePartialAssignment } from '../../schemas/zod/markers.js';

export const assignmentsController = {
  async getAll(req, res) {
    try {
      const assignments = await assignmentsModel.getAll();
      res.json(assignments);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getSettings(req, res) {
    try {
      const settings = await assignmentsModel.getBasesSettings();
      res.json(settings);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getById(req, res) {
    try {
      const { id } = req.params;
      const assignment = await assignmentsModel.getById(id);
      if (!assignment) {
        return res.status(404).json({ error: 'Asignación no encontrada' });
      }
      res.json(assignment);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async create(req, res) {
    try {
      const result = validateAssignment(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const assignment = await assignmentsModel.create(result.data);
      res.status(201).json(assignment);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async update(req, res) {
    try {
      const { id } = req.params;
      const result = validatePartialAssignment(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const assignment = await assignmentsModel.update(id, result.data);
      if (!assignment) {
        return res.status(404).json({ error: 'Asignación no encontrada' });
      }
      res.json(assignment);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async delete(req, res) {
    try {
      const { id } = req.params;
      const affected = await assignmentsModel.delete(id);
      if (affected === 0) {
        return res.status(404).json({ error: 'Asignación no encontrada' });
      }
      res.json({ message: 'Asignación eliminada correctamente' });
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },
};
