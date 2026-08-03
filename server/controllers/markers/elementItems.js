import { elementItemsModel } from '../../models/markers/elementItems.js';
import { validateElementItem, validatePartialElementItem } from '../../schemas/zod/markers.js';

export const elementItemsController = {
  async getAll(req, res) {
    try {
      const elementItems = await elementItemsModel.getAll();
      res.json(elementItems);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getById(req, res) {
    try {
      const { id } = req.params;
      const elementItem = await elementItemsModel.getById(id);
      if (!elementItem) {
        return res.status(404).json({ error: 'Elemento no encontrado' });
      }
      res.json(elementItem);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async create(req, res) {
    try {
      const result = validateElementItem(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const elementItem = await elementItemsModel.create(result.data);
      res.status(201).json(elementItem);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async update(req, res) {
    try {
      const { id } = req.params;
      const result = validatePartialElementItem(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const elementItem = await elementItemsModel.update(id, result.data);
      if (!elementItem) {
        return res.status(404).json({ error: 'Elemento no encontrado' });
      }
      res.json(elementItem);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async delete(req, res) {
    try {
      const { id } = req.params;
      const affected = await elementItemsModel.delete(id);
      if (affected === 0) {
        return res.status(404).json({ error: 'Elemento no encontrado' });
      }
      res.json({ message: 'Elemento eliminado correctamente' });
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },
};
