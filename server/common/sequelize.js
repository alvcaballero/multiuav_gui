import Sequelize from 'sequelize';
import { dbName, dbHost, dbPort, dbUser, dbPassword } from '../config/config';

import setupModels from '../database';

const USER = encodeURIComponent(dbUser);
const PASSWORD = encodeURIComponent(dbPassword);
const URI = `postgres://${USER}:${PASSWORD}@${dbHost}:${dbPort}/${dbName}`;

const sequelize = new Sequelize(URI, {
  dialect: 'postgres',
  logging: true,
});

setupModels(sequelize);

sequelize.sync();

export default sequelize;
