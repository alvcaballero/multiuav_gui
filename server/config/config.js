import dotenv from 'dotenv';

dotenv.config();

export const env = process.env.NODE_ENV || 'dev',
  port = process.env.PORT || 4000,
  CorsEnable = process.env.CORS_ENABLE === 'true' ? true : false,
  RosEnable = process.env.ROS_CONNECTION ? (process.env.ROS_CONNECTION === 'false' ? false : true) : true,
  FbEnable = process.env.FB_CONNECTION ? (process.env.FB_CONNECTION === 'false' ? false : true) : true,
  StreamServer = process.env.STREAM_SERVER === 'true' ? true : false, //if use mediamtx server in local host
  LocalGlyphs = process.env.LOCAL_GLYPHS === 'true' ? true : false, // if local server have a glyphs server
  NoElevation = process.env.NO_ELEVATION === 'true' ? true : false, // if local server have a glyphs server
  db = process.env.DB === 'true' ? true : false,
  dbUser = process.env.DB_USER,
  dbPassword = process.env.DB_PASSWORD,
  dbHost = process.env.DB_HOST,
  dbName = process.env.DB_NAME,
  dbPort = process.env.DB_PORT,
  planningServer = process.env.PLANNING_SERVER === 'true' ? true : false,
  planningHost = process.env.PLANNING_HOST,
  filesPath = process.env.Files_PATH ? process.env.Files_PATH : './data/',
  processThermalImg = process.env.Process_Thermal_Img === 'true' ? true : false,
  processThermalsSrc = process.env.Process_Program_src,
  extApp = process.env.EXT_APP === 'true' ? true : false,
  extAppUrl = process.env.EXT_APP_url || '',
  extAppUser = process.env.EXT_APP_user || '',
  extAppPWD = process.env.EXT_APP_pwd || '';

// data files
export const filesData = '../data/files.json';
export const devicesData = '../data/devices.json';
export const routesData = '../data/routes.json';
export const missionsData = '../data/missions.json';
export const missionsConfigData = '../data/missionConfig.yaml';
// config files
export const devicesMsg = '../config/devices/devices_msg.yaml';
export const messagesTypes = '../config/devices/messages.yaml';
