import * as ROSLIB from 'roslib';
import { logger } from '../../common/logger.js';

// Read-only introspection over the ROS computation graph.
// Every function here queries rosapi (or the ROS bridge) to discover
// what topics/services/actions exist and what their message types look like.
// No side effects: nothing here publishes, calls a service for effect, or
// mutates state. If a function makes ROS *do* something, it does not belong here.

export function getTopics(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  return new Promise((resolve, reject) => {
    ros.getTopics(
      (topics) => resolve(topics),
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

export async function getRosVersion(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');
  let servicemaster = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/get_ros_version',
    serviceType: 'rosapi_msgs/srv/GetROSVersion',
  });

  let request = {};
  return new Promise((resolve, rejects) => {
    servicemaster.callService(
      request,
      function (result) {
        logger.debug(`get_ros_version result: ${JSON.stringify(result)}`);
        resolve({ version: result.version, distro: result.distro });
      },
      function (error) {
        logger.error(`Error getting ros version: ${error.message}`);
        rejects(error);
      }
    );
  });
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

export async function getActionServers(ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  return new Promise((resolve, reject) => {
    ros.getActionServers(
      (servers) => resolve(servers),
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

export function Getservicehost(nameService, ros) {
  let servicehost = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/service_host',
    serviceType: 'rosapi/ServiceHost',
  });

  let request = { service: nameService };

  return new Promise((resolve, _rejects) => {
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

  return new Promise((resolve, _rejects) => {
    servicemaster.callService(request, function (result) {
      logger.debug(`Master IPs found: ${result.length}`);
      resolve(result);
    });
  });
}
