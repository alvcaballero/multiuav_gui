import dotenv from 'dotenv';

dotenv.config();

export const env = process.env.NODE_ENV || 'dev',
  port = process.env.PORT || 4000,
  dbUser = process.env.DB_USER,
  dbPassword = process.env.DB_PASSWORD,
  dbHost = process.env.DB_HOST,
  dbName = process.env.DB_NAME,
  dbPort = process.env.DB_PORT,
  planningServer = process.env.PLANNING_SERVER || false,
  planningHost = process.env.PLANNING_HOST,
  filesPath = process.env.Files_PATH,
  processThermalImg = process.env.Process_Img || false,
  processThermalsSrc = process.env.Process_src,
  extApp = process.env.EXT_APP || false,
  extAppUrl = process.env.EXT_APP_url || '',
  extAppUser = process.env.EXT_APP_user || '',
  extAppPWD = process.env.EXT_APP_pwd || '';
