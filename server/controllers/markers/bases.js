import { basesModel } from '../../models/markers/bases.js';
import { assignmentsModel } from '../../models/markers/assignments.js';
import { validateBase, validatePartialBase } from '../../schemas/zod/markers.js';

export const basesController = {
  async getAll(req, res) {
    try {
      const bases = await basesModel.getAll();
      res.json(bases);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getAssigned(req, res) {
    try {
      const bases = await assignmentsModel.getBaseswithAssignments();
      res.json(bases);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getById(req, res) {
    try {
      const { id } = req.params;
      const base = await basesModel.getById(id);
      if (!base) {
        return res.status(404).json({ error: 'Base no encontrada' });
      }
      res.json(base);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async create(req, res) {
    try {
      const result = validateBase(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const base = await basesModel.create(result.data);
      res.status(201).json(base);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async update(req, res) {
    try {
      const { id } = req.params;
      const result = validatePartialBase(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const base = await basesModel.update(id, result.data);
      if (!base) {
        return res.status(404).json({ error: 'Base no encontrada' });
      }
      res.json(base);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async delete(req, res) {
    try {
      const { id } = req.params;
      const affected = await basesModel.delete(id);
      if (affected === 0) {
        return res.status(404).json({ error: 'Base no encontrada' });
      }
      res.json({ message: 'Base eliminada correctamente' });
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },
};
