import { rosModel } from '../models/ros.js';
import { RosEnable } from '../config/config.js';
export class rosController {
  static async getTopics(req, res) {
    try {
      const response = await rosModel.getTopics();
      res.json(response);
    } catch (error) {
      console.error('Error getting topics:', error);
      res.status(500).json({ error:'Error getting topics: ' + error });
    }
  }
  static async getTopicType(req, res) {
    const { topic } = req.query;  
    if (!topic || typeof topic !== 'string') {
      return res.status(400).json({ error: 'El parámetro "topic" es obligatorio y debe ser una cadena' });
    }
    try {
      const response = await rosModel.getTopicType(topic);
      res.json(response);
    } catch (error) {
      console.error('Error getting topic type:', error);
      res.status(500).json({ error: 'Failed to get topic type: ' + error });
    }
  }
  static async getMessageDetails(req, res) {
    const { type } = req.query;
    if (!type || typeof type !== 'string') {
      return res.status(400).json({ error: 'El parámetro "type" es obligatorio y debe ser una cadena' });
    }
    try {
      const response = await rosModel.getMessageDetails(type);
      res.json(response);
    } catch (error) {
      console.error('Error getting message details:', error);
      res.status(500).json({ error: 'Failed to get message details: ' + error });
    }
  }
  static async getPublishers(req, res) {
    const { topic } = req.query;
    if (!topic || typeof topic !== 'string') {
      return res.status(400).json({ error: 'El parámetro "topic" es obligatorio y debe ser una cadena' });
    }
    try {
      const response = await rosModel.getPublishers(topic);
      res.json(response);
    } catch (error) {
      console.error('Error getting publishers:', error);
      res.status(500).json({ error: 'Failed to get publishers' + error });
    }
  }

  static async getServices(req, res) {
    try {
      const response = await rosModel.getServices();
      res.json(response);
    } catch (error) {
      console.error('Error getting services:', error);
      res.status(500).json({ error: 'Failed to get services' + error });
    }
  }
  static async pubTopicOnce(req, res) {
    try {
      const response = await rosModel.PubRosMsg(req.body);
      res.json(response);
    } catch (error) {
      console.error('Error publishing message:', error);
      res.status(500).json({ error: 'Failed to publish message' + error });
    }
  }
  static async subscribeOnce(req, res) {
    try {
      const response = await rosModel.subscribeOnce(req.query);
      res.json(response);
    } catch (error) {
      console.error('Error subscribing to topic:', error);
      res.status(500).json({ error: 'Failed to subscribe to topic: ' + error.message });
    }
  }

  static async getListMaster(req, res) {
    console.log('controller get all');
    const response = await rosModel.getListMaster();
    res.json(response);
  }

  static async subscribeDevice({ id, name, category, camera, watch_bound = true, bag = false }) {
    return RosEnable ? await rosModel.subscribeDevice({ id, name, category, camera, watch_bound, bag }) : null;
  }
  static async unsubscribeDevice(id) {
    console.log('unsuscribe controller');
    return RosEnable ? await rosModel.unsubscribeDevice(id) : null;
  }
  static async callService(message) {
    if (!RosEnable) return { state: 'error', message: 'ROS connection is disabled' };
    let response = await rosModel.callService(message);
    return response;
  }

  static getServerStatus() {
    return rosModel.serverStatus();
  }
}
