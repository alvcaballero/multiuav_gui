import { elementTypesModel } from '../../models/markers/elementTypes.js';
import { validateElementType, validatePartialElementType } from '../../schemas/zod/markers.js';

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
};
