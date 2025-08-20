import Sequelize, { Op } from 'sequelize';

import { db, dbType, dbName, dbHost, dbPort, dbUser, dbPassword } from '../config/config.js';
import { setupModels } from '../schemas/database/index.js';
import logger from './logger.js';

logger.info('use db is ' + db);

let sequelizeConfig = {
  dialect: 'sqlite',
  storage: 'data/sequelize.sqlite',
  logging: false,
  pool: { max: 1, idle: Infinity, maxUses: Infinity },
};

if (db === 'true') {
  if (dbType === 'postgres') {
    sequelizeConfig = {
      dialect: 'postgres',
      database: dbName,
      user: dbUser,
      password: dbPassword,
      host: dbHost,
      port: dbPort,
      logging: false,
    };
  }
  if (dbType === 'mysql') {
    sequelizeConfig = {
      dialect: 'mysql',
      database: dbName,
      user: dbUser,
      password: dbPassword,
      host: dbHost,
      port: dbPort,
      logging: false,
    };
  }
}

const sequelize = new Sequelize(sequelizeConfig);

try {
  await sequelize.authenticate();
  logger.info('DB Connection successfully.');
} catch (error) {
  logger.error('Unable to connect to the database: ', error);
}

setupModels(sequelize);

try {
  await sequelize.sync({ force: false });
  logger.info('DB  models were synchronized successfully.');
} catch (error) {
  logger.error('DB Unable to create tables : ', error);
}

export default sequelize;
export { Op };
