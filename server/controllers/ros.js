import { rosModel } from '../models/ros/ros.js';
import { RosEnable } from '../config/config.js';
import logger from '../common/logger.js';
export class rosController {
  static async getTopics(req, res) {
    try {
      const response = await rosModel.getTopics();
      res.json(response);
    } catch (error) {
      logger.error(`Error getting topics: ${error}`);
      res.status(500).json({ error: 'Error getting topics: ' + error });
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
      logger.error(`Error getting topic type: ${error}`);
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
      logger.error(`Error getting message details: ${error}`);
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
      logger.error(`Error getting publishers: ${error}`);
      res.status(500).json({ error: 'Failed to get publishers' + error });
    }
  }

  static async getServices(req, res) {
    try {
      const response = await rosModel.getServices();
      res.json(response);
    } catch (error) {
      logger.error(`Error getting services: ${error}`);
      res.status(500).json({ error: 'Failed to get services' + error });
    }
  }
  static async getServicesType(req, res) {
    const { service } = req.query;
    if (!service || typeof service !== 'string') {
      return res.status(400).json({ error: 'El parámetro "service" es obligatorio y debe ser una cadena' });
    }
    try {
      const response = await rosModel.getServicesType(service);
      res.json(response);
    } catch (error) {
      logger.error(`Error getting services type: ${error}`);
      res.status(500).json({ error: 'Failed to get services type' + error });
    }
  }
  static async getServiceRequestDetails(req, res) {
    const { type } = req.query;
    if (!type || typeof type !== 'string') {
      return res.status(400).json({ error: 'El parámetro "type" es obligatorio y debe ser una cadena' });
    }
    try {
      const response = await rosModel.getServiceRequestDetails(type);
      res.json(response);
    } catch (error) {
      logger.error(`Error getting service details: ${error}`);
      res.status(500).json({ error: 'Failed to get service details: ' + error });
    }
  }

  static async getServiceResponseDetails(req, res) {
    const { type } = req.query;
    if (!type || typeof type !== 'string') {
      return res.status(400).json({ error: 'El parámetro "type" es obligatorio y debe ser una cadena' });
    }
    try {
      const response = await rosModel.getServiceRequestDetails(type);
      res.json(response);
    } catch (error) {
      logger.error(`Error getting service details: ${error}`);
      res.status(500).json({ error: 'Failed to get service details: ' + error });
    }
  }
  static async getActionServers(req, res) {
    try {
      const response = await rosModel.getActionServers();
      res.json(response);
    } catch (error) {
      logger.error(`Error getting action servers: ${error.message}`);
      res.status(500).json({ error: 'Failed to get action servers: ' + error });
    }
  }
  static async sendActionGoal(req, res) {
    try {
      const response = await rosModel.sendActionGoal(req.body);
      res.json(response);
    } catch (error) {
      logger.error(`Error calling action: ${error.message}`);
      res.status(400).json({ error: error.message });
    }
  }

  // Status for every action of a device, by device name (internal callers).
  static getActionStatusByName(name) {
    return rosModel.getActionStatusByName(name);
  }

  static async getActionStatusHandler(req, res) {
    const { uav_id, type } = req.query;
    if (!uav_id) return res.status(400).json({ error: 'uav_id is required' });
    try {
      // type given → that specific action; otherwise every action of the device
      res.json(await rosModel.getActionStatus({ uav_id, type }));
    } catch (error) {
      res.status(400).json({ error: error.message });
    }
  }

  static async cancelActionHandler(req, res) {
    const { uav_id, type } = req.body;
    if (!uav_id || !type) return res.status(400).json({ error: 'uav_id and type are required' });
    try {
      res.json(await rosModel.cancelAction({ uav_id, type }));
    } catch (error) {
      res.status(400).json({ error: error.message });
    }
  }

  // ─── Primitive-layer handlers: caller passes the raw ROS action name/type ───
  // `action` here is the full action-server name (e.g. /agv_1/navigate_to_pose).

  static async sendRosActionGoalHandler(req, res) {
    const { action, actionType, message, target, timeout, blocking } = req.body;
    if (!action || !actionType) return res.status(400).json({ error: 'action and actionType are required' });
    try {
      const response = await rosModel.sendRosActionGoal({
        actionServerName: action,
        actionType,
        message,
        target,
        timeout,
        blocking,
      });
      res.json(response);
    } catch (error) {
      logger.error(`Error calling ros action: ${error.message}`);
      res.status(400).json({ error: error.message });
    }
  }

  static getRosActionStatusHandler(req, res) {
    const { action } = req.query;
    if (!action) return res.status(400).json({ error: 'action is required' });
    res.json(rosModel.getRosActionStatus({ actionServerName: action }));
  }

  static cancelRosActionHandler(req, res) {
    const { action } = req.body;
    if (!action) return res.status(400).json({ error: 'action is required' });
    res.json(rosModel.cancelRosAction({ actionServerName: action }));
  }

  static async callRosService(req, res) {
    try {
      const response = await rosModel.callRosService(req.body);
      const byteSize = JSON.stringify(response).length;
      logger.debug(`Service call response: ${byteSize} bytes, keys: ${Object.keys(response || {}).join(', ')}`);
      res.json(response);
    } catch (error) {
      logger.error(`Error calling service: ${error.message}`);
      res.status(500).json({ error: 'Failed to call service: ' + error.message });
    }
  }
  static async pubTopicOnce(req, res) {
    try {
      const response = await rosModel.PubRosMsg(req.body);
      res.json(response);
    } catch (error) {
      logger.error(`Error publishing message: ${error.message}`);
      res.status(500).json({ error: 'Failed to publish message' + error });
    }
  }
  static async subscribeOnce(req, res) {
    try {
      const response = await rosModel.subscribeOnce(req.query);
      res.json(response);
    } catch (error) {
      logger.error(`Error subscribing to topic: ${error.message}`);
      res.status(500).json({ error: 'Failed to subscribe to topic: ' + error.message });
    }
  }

  static async getListMaster(req, res) {
    logger.debug('getListMaster');
    const response = await rosModel.getListMaster();
    res.json(response);
  }

  static async subscribeDevice({ id, name, category, camera, watch_bound = true, bag = false }) {
    return RosEnable ? await rosModel.subscribeDevice({ id, name, category, camera, watch_bound, bag }) : null;
  }
  static async unsubscribeDevice(id) {
    logger.debug(`unsubscribeDevice id=${id}`);
    return RosEnable ? await rosModel.unsubscribeDevice(id) : null;
  }
  static async callService(message) {
    if (!RosEnable) return { state: 'error', message: 'ROS connection is disabled' };
    let response = await rosModel.callService(message);
    return response;
  }

  // Internal (non-HTTP) counterpart of sendActionGoal, used by commandsModel.standarCommand.
  static async sendActionGoal({ uav_id, type, message, target, timeout, blocking }) {
    if (!RosEnable) return { state: 'error', message: 'ROS connection is disabled' };
    return await rosModel.sendActionGoal({ uav_id, type, message, target, timeout, blocking });
  }

  static getServerStatus() {
    return rosModel.serverStatus();
  }
}
