import dotenv from 'dotenv';

dotenv.config();

export const env = process.env.NODE_ENV || 'dev',
  port = process.env.PORT || 4000,
  StreamServer = process.env.STREAM_SERVER === 'true' ? true : false, //if use mediamtx server in local host
  LocalGlyphs = process.env.LOCAL_GLYPHS === 'true' ? true : false, // if local server have a glyphs server
  db = process.env.DB === 'true' ? true : false,
  dbUser = process.env.DB_USER,
  dbPassword = process.env.DB_PASSWORD,
  dbHost = process.env.DB_HOST,
  dbName = process.env.DB_NAME,
  dbPort = process.env.DB_PORT,
  planningServer = process.env.PLANNING_SERVER === 'true' ? true : false,
  planningHost = process.env.PLANNING_HOST,
  filesPath = process.env.Files_PATH,
  processThermalImg = process.env.Process_Img === 'true' ? true : false,
  processThermalsSrc = process.env.Process_src,
  extApp = process.env.EXT_APP === 'true' ? true : false,
  extAppUrl = process.env.EXT_APP_url || '',
  extAppUser = process.env.EXT_APP_user || '',
  extAppPWD = process.env.EXT_APP_pwd || '';
