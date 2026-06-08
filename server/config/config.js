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
export const useExternalDb = process.env.DB === 'true';
export const dbType = process.env.DB_TYPE;
export const dbUser = process.env.DB_USER;
export const dbPassword = process.env.DB_PASSWORD;
export const dbHost = process.env.DB_HOST;
export const dbName = process.env.DB_NAME;
export const dbPort = process.env.DB_PORT;
export const planningServer = process.env.PLANNING_SERVER === 'true';
export const planningHost = process.env.PLANNING_HOST;
export const filesPath = process.env.FILES_PATH ?? './data/';
export const processThermalImg = process.env.PROCESS_THERMAL_IMG === 'true';
export const processThermalsSrc = process.env.PROCESS_PROGRAM_SRC;
export const extApp = process.env.EXT_APP === 'true';
export const extAppUrl = process.env.EXT_APP_URL || '';
export const extAppUser = process.env.EXT_APP_USER || '';
export const extAppPWD = process.env.EXT_APP_PWD || '';
export const LLM = process.env.LLM === 'true';
export const LLMProvider = process.env.LLM_PROVIDER || 'openai';
export const LLMApiKeys = {
  openai: process.env.LLM_OPENAI_API_KEY || '',
  gemini: process.env.LLM_GEMINI_API_KEY || '',
  anthropic: process.env.LLM_ANTHROPIC_API_KEY || '',
  ollama: process.env.LLM_OLLAMA_API_KEY || 'http://localhost:11434',
};
export const MCPenable = process.env.MCP_ENABLE === 'true'; // Model Context Protocol

const VALID_MCP_TRANSPORTS = ['stdio', 'http', 'sse'];
let _MCPconfig = {};
const raw = process.env.MCP_CONFIG;
if (raw) {
  try {
    _MCPconfig = JSON.parse(raw);
    if (!VALID_MCP_TRANSPORTS.includes(_MCPconfig.transport)) {
      throw new Error(`Unknown transport '${_MCPconfig.transport}'. Valid: ${VALID_MCP_TRANSPORTS.join(', ')}`);
    }
    logger.info('MCP config loaded', _MCPconfig);
  } catch (err) {
    logger.error('Invalid MCP_CONFIG:', err.message);
    _MCPconfig = {};
  }
}
export const MCPconfig = _MCPconfig; // MCP configuration file
// data files
export const filesData = '../data/files.json';
export const devicesData = '../data/devices.json';
export const routesData = '../data/routes.json';
export const missionsData = '../data/missions.json';
export const missionsConfigData = '../data/missionConfig.yaml';
// intervals
export const mapLatitude = Number(process.env.MAP_LATITUDE) || 37.19384681403371;
export const mapLongitude = Number(process.env.MAP_LONGITUDE) || -6.702598762315071;
export const mapZoom = Number(process.env.MAP_ZOOM) || 15;
export const WS_PING_INTERVAL_MS = Number(process.env.WS_PING_INTERVAL_MS) || 30000;
export const WS_POSITIONS_INTERVAL_MS = Number(process.env.WS_POSITIONS_INTERVAL_MS) || 500;
export const WS_STATE_INTERVAL_MS = Number(process.env.WS_STATE_INTERVAL_MS) || 10000;
export const ROS_URL = process.env.ROS_URL || 'ws://127.0.0.1:9090';
export const ROS_RECONNECT_INTERVAL_MS = Number(process.env.ROS_RECONNECT_INTERVAL_MS) || 30000;
export const DEVICE_CHECK_INTERVAL_MS = Number(process.env.DEVICE_CHECK_INTERVAL_MS) || 5000;
export const DEVICE_UPDATE_INTERVAL_MS = Number(process.env.DEVICE_UPDATE_INTERVAL_MS) || 2000;
export const DEVICE_TIMEOUT_MS = Number(process.env.DEVICE_TIMEOUT_MS) || 30000;
// config files
export const devicesMsg = '../config/devices/devices_msg.yaml';
export const missionSchema = '../config/devices/mission_schema.yaml';
export const messagesTypes = '../config/devices/messages.yaml';
