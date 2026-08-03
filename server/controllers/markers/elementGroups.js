import { elementGroupsModel } from '../../models/markers/elementGroups.js';
import { elementItemsModel } from '../../models/markers/elementItems.js';
import { validateElementGroup, validatePartialElementGroup } from '../../schemas/zod/markers.js';

export const elementGroupsController = {
  async getAll(req, res) {
    try {
      const elementGroups = await elementGroupsModel.getAll();
      res.json(elementGroups);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getById(req, res) {
    try {
      const { id } = req.params;
      const elementGroup = await elementGroupsModel.getById(id);
      if (!elementGroup) {
        return res.status(404).json({ error: 'Grupo de elementos no encontrado' });
      }
      res.json(elementGroup);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async getItems(req, res) {
    try {
      const { id } = req.params;
      const items = await elementItemsModel.findByGroup(id);
      res.json(items);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async create(req, res) {
    try {
      const result = validateElementGroup(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const elementGroup = await elementGroupsModel.create(result.data);
      res.status(201).json(elementGroup);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async update(req, res) {
    try {
      const { id } = req.params;
      const result = validatePartialElementGroup(req.body);
      if (!result.success) {
        return res.status(400).json({ error: result.error.message });
      }
      const elementGroup = await elementGroupsModel.update(id, result.data);
      if (!elementGroup) {
        return res.status(404).json({ error: 'Grupo de elementos no encontrado' });
      }
      res.json(elementGroup);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },

  async delete(req, res) {
    try {
      const { id } = req.params;
      const [affected] = await elementGroupsModel.delete(id);
      if (affected === 0) {
        return res.status(404).json({ error: 'Grupo de elementos no encontrado' });
      }
      res.json({ message: 'Grupo de elementos eliminado correctamente' });
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  },
};
