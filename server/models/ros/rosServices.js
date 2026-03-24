import * as ROSLIB from 'roslib';
import { readDataFile } from '../../common/utils.js';
import { devicesController } from '../../controllers/devices.js';
import { missionController } from '../../controllers/mission.js';
import { encodeRosSrv } from './rosEncode.js';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { ROS2GoalActionClient } from './rosActionClient.js';
import logger, { logHelpers } from '../../common/logger.js';

const devices_msg = readDataFile('../config/devices/devices_msg.yaml');

// Module-level list used by GCSServicesMission / GCSunServicesMission
const service_list = [];

export async function callRosService({ service, messageType, message }, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const services = await getServices(ros);
  if (!services.includes(service)) {
    logger.debug(`Available services: ${JSON.stringify(services)}`);
    throw new Error(`Service '${service}' not available`);
  }

  const topicType = await getServicesType(service, ros);
  const auxtopicType = topicType.replace('/srv/', '/');
  if (!(topicType == messageType || auxtopicType == messageType)) {
    throw new Error(`Service type mismatch: expected '${topicType}', got '${messageType}'`);
  }

  const responseSrvStructure = await getServiceRequestDetails(messageType, ros);
  logger.debug(`Service request structure: ${JSON.stringify(responseSrvStructure)}`);
  const srvStructure = responseSrvStructure.typedefs || [];

  if (!srvStructure || srvStructure.length === 0) {
    throw new Error(`No structure found for service type '${messageType}'`);
  }

  const typeMap = buildTypeMap(srvStructure);

  validateRosMsg(messageType, message, typeMap);

  let Message = new ROSLIB.Service({
    ros: ros,
    name: service,
    serviceType: messageType,
  });

  let MsgRequest = message ? message : {};

  return new Promise((resolve, rejects) => {
    Message.callService(
      MsgRequest,
      (result) => resolve(result),
      (error) => rejects(error)
    );
  });
}

export async function callService({ uav_id, type, request }, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  let device = await devicesController.getDevice(uav_id);
  const { name, category } = device;

  if (!devices_msg[category]['services'].hasOwnProperty(type)) {
    return { state: 'warning', msg: type + ' to:' + name + ' dont have this service' };
  }

  let msgType = devices_msg[category]['services'][type]['serviceType'];
  let myRequest = encodeRosSrv({ type, msg: request, msgType: msgType });
  const service = `/${name}${devices_msg[category]['services'][type]['name']}`;
  try {
    const response = await callRosService({ service, messageType: msgType, message: myRequest }, ros);
    if (response.success || response.result) {
      return { state: 'success', msg: type + ' to ' + name + ' ok' };
    } else {
      return { state: 'error', msg: type + ' to  ' + name + ' error' };
    }
  } catch (error) {
    logger.error(`Error calling service: ${error.message}`);
    return { state: 'error', msg: 'Failed to call service: ' + error.message };
  }
}

export function getTopics(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  return new Promise((resolve, reject) => {
    ros.getTopics(
      (topics) => resolve(topics),
      (error) => reject(error)
    );
  });
}

export function getServices(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  return new Promise((resolve, reject) => {
    ros.getServices(
      (services) => resolve(services),
      (error) => reject(error)
    );
  });
}

export async function getServicesType(service, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  logger.debug(`Getting service type for ${service}`);
  return new Promise((resolve, reject) => {
    ros.getServiceType(
      service,
      (serviceType) => resolve(serviceType),
      (error) => reject(error)
    );
  });
}

export async function getServiceRequestDetails(type, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  return new Promise((resolve, reject) => {
    ros.getServiceRequestDetails(
      type,
      (serviceDetails) => {
        if (!serviceDetails || serviceDetails.length === 0) {
          reject(new Error('Service type not found'));
        }
        resolve(serviceDetails);
      },
      (error) => reject(error)
    );
  });
}

export async function getServiceResponseDetails(type, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  return new Promise((resolve, reject) => {
    ros.getServiceResponseDetails(
      type,
      (serviceDetails) => {
        if (!serviceDetails || serviceDetails.length === 0) {
          reject(new Error('Service type not found'));
        }
        resolve(serviceDetails);
      },
      (error) => reject(error)
    );
  });
}

export function getTopicType(topic, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  logger.debug(`Getting topic type for ${topic}`);
  return new Promise((resolve, reject) => {
    ros.getTopicType(
      topic,
      (topicType) => resolve(topicType),
      (error) => reject(error)
    );
  });
}

export function getMessageDetails(message, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  return new Promise((resolve, reject) => {
    ros.getMessageDetails(
      message,
      (messageDetails) => {
        if (!messageDetails || (Array.isArray(messageDetails) && messageDetails.length === 0)) {
          reject(new Error('Message type not found'));
        }
        resolve(messageDetails);
      },
      (error) => reject(error)
    );
  });
}

export async function getRosVersion(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  let servicemaster = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/get_ros_version',
    serviceType: 'rosapi_msgs/srv/GetRosVersion',
  });

  let request = {};
  return new Promise((resolve, rejects) => {
    servicemaster.callService(
      request,
      function (result) {
        resolve(result.ros_version);
      },
      function (error) {
        logger.error(`Error getting ros version: ${error.message}`);
        rejects(error);
      }
    );
  });
}

export async function getPublishers(topic, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  let servicemaster = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/publishers',
    serviceType: 'rosapi_msgs/srv/Publishers',
  });

  let request = { topic: topic };

  return new Promise((resolve, rejects) => {
    servicemaster.callService(
      request,
      function (result) {
        logger.debug(`Publishers for topic '${topic}': ${JSON.stringify(result)}`);
        resolve(result.publishers);
      },
      function (error) {
        logger.error(`Error getting publishers: ${error}`);
        rejects(error);
      }
    );
  });
}

export async function PubRosMsg(params, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const { topic, messageType, message } = params;

  const msgStructure = await getMessageDetails(messageType, ros);

  const typeMap = buildTypeMap(msgStructure);

  validateRosMsg(messageType, message, typeMap);

  const subscribers = await getTopics(ros);
  if (!subscribers.topics.includes(topic)) {
    throw new Error(`No subscribers found for topic '${topic}'`);
  }

  const pub = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: messageType,
  });
  const rosMsg = encodeRosSrv({ type: '', msg: message, msgType: messageType });
  if (!rosMsg) {
    throw new Error('ROS message is empty or invalid');
  }

  pub.on('warning', function (warning) {
    logger.warn(`ROS publish warning: ${warning}`);
  });

  pub.publish(rosMsg);
  return { topic: topic, msgType: messageType, msg: 'Message published successfully' };
}

export async function subscribeOnce({ topic, messageType, timeout = 2000 }, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const topics = await getTopics(ros);
  if (!topics.topics.includes(topic)) {
    return Promise.reject(new Error(`Topic '${topic}' does not exist`));
  }

  const publisher = await getPublishers(topic, ros);
  if (publisher.length === 0) {
    logger.warn(
      `Warning: /rosapi/publishers returned empty list for topic '${topic}', but topic exists. Proceeding anyway...`
    );
  }

  const sub = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: messageType,
  });

  return new Promise((resolve, reject) => {
    const handler = (message) => {
      sub.unsubscribe();
      resolve(message);
    };

    sub.subscribe(handler);

    setTimeout(() => {
      sub.unsubscribe();
      reject(new Error('Timeout exceeded'));
    }, timeout);
  });
}

export function serviceServer({ serviceName, serviceType, callback }, ros) {
  const service = new ROSLIB.Service({
    ros: ros,
    name: serviceName,
    serviceType: serviceType,
  });
  service.advertise(function (request, response) {
    callback(request, response);
  });
  return service;
}

export function GCSServicesMission(ros) {
  const gcs_services = [
    {
      name: 'ServiceFinishMission',
      serviceName: '/GCS/FinishMission',
      serviceType: 'aerialcore_common/finishMission',
      callback: function (request, response) {
        logger.debug(`Service finish mission callback: ${JSON.stringify(request)}`);
        if (request.hasOwnProperty('uav_id')) {
          missionController.deviceFinishMission({ name: request.uav_id });
        }
        Object.assign(response, { success: true, msg: 'Set successfully' });
        return true;
      },
    },
    {
      name: 'ServiceDownload',
      serviceName: '/GCS/FinishDownload',
      serviceType: 'aerialcore_common/finishGetFiles',
      callback: function (request, response) {
        logger.debug(`Service finish download files callback: ${JSON.stringify(request)}`);
        if (request.hasOwnProperty('uav_id')) {
          missionController.deviceFinishSyncFiles({ name: request.uav_id });
        }
        Object.assign(response, { success: true, msg: 'Set successfully' });
        return true;
      },
    },
  ];

  for (let srv of gcs_services) {
    service_list[srv.name] = serviceServer(
      {
        serviceName: srv.serviceName,
        serviceType: srv.serviceType,
        callback: srv.callback,
      },
      ros
    );
  }
}

export function GCSunServicesMission() {
  for (let srv of service_list) {
    service_list[srv.name].unadvertise();
    delete service_list[srv.name];
  }
}

export async function getActionServer(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  return new Promise((resolve, reject) => {
    ros.getActionServers(
      (actions) => resolve(actions),
      (error) => reject(error)
    );
  });
}

export async function getActionGoalmsg(actionServer, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  let servicemaster = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/action_goal_details',
    serviceType: 'rosapi_msgs/srv/ActionGoalDetails',
  });

  let request = { type: actionServer };

  return new Promise((resolve, rejects) => {
    servicemaster.callService(
      request,
      function (result) {
        resolve(result.publishers);
      },
      function (error) {
        logger.error(`Error getting action goal message: ${error}`);
        rejects(error);
      }
    );
  });
}

export async function sendActionGoal(args, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  const { action, actionType, message } = args;
  logger.info(`Sending goal to action server '${action}' of type '${actionType}' with message: ${JSON.stringify(message)}`);

  let newClient = new ROSLIB.Action({
    ros: ros,
    name: action,
    actionType: actionType,
  });

  let goal_id = newClient.sendGoal(
    message,
    (result) => {
      logger.info(`Action result: ${JSON.stringify(result)}`);
      if (result.result && result.status === 4) {
        logger.info('Navigation completed');
      } else {
        logger.warn('Navigation failed or was cancelled');
      }
    },
    (feedback) => {
      logger.debug(`Action feedback: ${JSON.stringify(feedback)}`);
    },
    (error) => {
      logger.error(`Action goal failed: ${error}`);
    }
  );
  logger.info(`Goal sent with ID: ${goal_id}`);

  const goalHandle = { id: 'a' };
  return { state: 'success', msg: 'Action goal sent successfully', goalId: goalHandle.id };
}

export async function cancelActionGoal(args, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  const { action, actionType, goalId } = args;

  const nav2Client = new ROS2GoalActionClient(ros, action, actionType, true);
  nav2Client.cancelGoal(goalId);
  return { state: 'success', msg: 'Action goal canceled successfully' };
}

export async function getActionServers(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  return new Promise((resolve, reject) => {
    ros.getActionServers(
      (servers) => resolve(servers),
      (error) => reject(error)
    );
  });
}

export function Getservicehost(nameService, ros) {
  let servicehost = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/service_host',
    serviceType: 'rosapi/ServiceHost',
  });

  let request = { service: nameService };

  return new Promise((resolve, rejects) => {
    servicehost.callService(request, function (result) {
      resolve(result.host);
    });
  });
}

export function getListMaster(ros) {
  let servicemaster = new ROSLIB.Service({
    ros: ros,
    name: '/master_discovery/list_masters',
    serviceType: 'multimaster_msgs_fkie/DiscoverMasters',
  });

  let request = {};

  return new Promise((resolve, rejects) => {
    servicemaster.callService(request, function (result) {
      logger.debug(`Master IPs found: ${result.length}`);
      resolve(result);
    });
  });
}
