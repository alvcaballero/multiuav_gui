import Sequelize, { Op } from 'sequelize';

import { useExternalDb, dbType, dbName, dbHost, dbPort, dbUser, dbPassword } from '../config/config.js';
import { setupModels } from '../schemas/database/index.js';
import logger from './logger.js';

logger.info('useExternalDb is ' + useExternalDb);

let sequelizeConfig = {
  dialect: 'sqlite',
  storage: 'data/sequelize.sqlite',
  logging: false,
  pool: { max: 1, idle: Infinity, maxUses: Infinity },
};

if (useExternalDb) {
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

// Additive column migrations — safe to run on every startup.
// Add new ALTER TABLE statements here when adding columns to existing tables.
const migrations = [
  `ALTER TABLE ChatMessage ADD COLUMN hidden INTEGER NOT NULL DEFAULT 0`,
];

for (const sql of migrations) {
  try {
    await sequelize.query(sql);
  } catch (e) {
    // "duplicate column name" means the column already exists — skip silently
    if (!e.message?.includes('duplicate column')) {
      logger.error(`Migration failed: ${sql}`, e.message);
    }
  }
}

export default sequelize;
export { Op };
