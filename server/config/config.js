import dotenv from 'dotenv';
import logger from '../common/logger.js';

dotenv.config();

export const env = process.env.NODE_ENV || 'dev';
export const port = Number(process.env.PORT) || 4000;
export const CorsEnable = process.env.CORS_ENABLE === 'true';
export const RosEnable =
  typeof process.env.ROS_CONNECTION === 'undefined' ? true : process.env.ROS_CONNECTION !== 'false';
export const FbEnable = typeof process.env.FB_CONNECTION === 'undefined' ? true : process.env.FB_CONNECTION !== 'false';
export const StreamServer = process.env.STREAM_SERVER === 'true'; //if use mediamtx server in local host
export const LocalGlyphs = process.env.LOCAL_GLYPHS === 'true'; // if local server have a glyphs server
export const NoElevation = process.env.NO_ELEVATION === 'true'; // if local server have a glyphs server
export const db = process.env.DB === 'true';
export const dbType = process.env.DB_TYPE;
export const dbUser = process.env.DB_USER;
export const dbPassword = process.env.DB_PASSWORD;
export const dbHost = process.env.DB_HOST;
export const dbName = process.env.DB_NAME;
export const dbPort = process.env.DB_PORT;
export const planningServer = process.env.PLANNING_SERVER === 'true';
export const planningHost = process.env.PLANNING_HOST;
export const filesPath = process.env.Files_PATH ? process.env.Files_PATH : './data/';
export const processThermalImg = process.env.Process_Thermal_Img === 'true';
export const processThermalsSrc = process.env.Process_Program_src;
export const extApp = process.env.EXT_APP === 'true';
export const extAppUrl = process.env.EXT_APP_url || '';
export const extAppUser = process.env.EXT_APP_user || '';
export const extAppPWD = process.env.EXT_APP_pwd || '';
export const LLM = process.env.LLM === 'true';
export const LLMType = process.env.LLM_TYPE || 'gemini'; // gemini or openai
export const LLMApiKey = process.env.LLM_API_KEY || '';
export const MCPenable = process.env.MCP_ENABLE === 'true'; // Model Context Protocol

let _MCPconfig = {};
try {
  const raw = process.env.MCP_CONFIG;
  _MCPconfig = raw ? JSON.parse(raw) : {};
  logger.info('_MCPconfig', _MCPconfig);
  if (typeof _MCPconfig.transport === 'undefined') {
    throw new Error(`Unknown transport: ${_MCPconfig.transport}`);
  }
} catch (err) {
  // invalid JSON in MCP_CONFIG env var — fallback to empty object
  logger.error('Invalid MCP_CONFIG JSON:', err);
  _MCPconfig = {};
}
export const MCPconfig = _MCPconfig; // MCP configuration file
// data files
export const filesData = '../data/files.json';
export const devicesData = '../data/devices.json';
export const routesData = '../data/routes.json';
export const missionsData = '../data/missions.json';
export const missionsConfigData = '../data/missionConfig.yaml';
// config files
export const devicesMsg = '../config/devices/devices_msg.yaml';
export const messagesTypes = '../config/devices/messages.yaml';
