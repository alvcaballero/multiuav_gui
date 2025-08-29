import { rosModel } from '../models/ros.js';
import { RosEnable } from '../config/config.js';
export class rosController {
  static async getTopics(req, res) {
    try {
      const response = await rosModel.getTopics();
      res.json(response);
    } catch (error) {
      console.error('Error getting topics:', error);
      res.status(500).json({ error: 'Failed to get topics' });
    }
  }
  static async getTopicType(req, res) {
    try {
      const response = await rosModel.getTopicType(req.query.topic);
      res.json(response);
    } catch (error) {
      console.error('Error getting topic type:', error);
      res.status(500).json({ error: 'Failed to get topic type' });
    }
  }
  static async getMessageDetails(req, res) {
    try {
      const response = await rosModel.getMessageDetails(req.query.type);
      res.json(response);
    } catch (error) {
      console.error('Error getting message details:', error);
      res.status(500).json({ error: 'Failed to get message details' });
    }
  }
  static async getPublishers(req, res) {
    try {
      const response = await rosModel.getPublishers(req.query.topic);
      res.json(response);
    } catch (error) {
      console.error('Error getting publishers:', error);
      res.status(500).json({ error: 'Failed to get publishers' });
    }
  }

  static async getServices(req, res) {
    try {
      const response = await rosModel.getServices();
      res.json(response);
    } catch (error) {
      console.error('Error getting services:', error);
      res.status(500).json({ error: 'Failed to get services' });
    }
  }
  static async pubTopicOnce(req, res) {
    try {
      const response = await rosModel.PubRosMsg(req.body);
      res.json(response);
    } catch (error) {
      console.error('Error publishing message:', error);
      res.status(500).json({ error: 'Failed to publish message' });
    }
  }
  static async subscribeOnce(req, res) {
    try {
      const response = await rosModel.subscribeOnce(req.query);
      res.json(response);
    } catch (error) {
      console.error('Error subscribing to topic:', error);
      res.status(500).json({ error: 'Failed to subscribe to topic' });
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
