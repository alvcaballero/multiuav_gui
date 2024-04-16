import Sequelize from 'sequelize';

import { dbName, dbHost, dbPort, dbUser, dbPassword } from '../config/config.js';
import { setupModels } from '../database/index.js';

const USER = encodeURIComponent(dbUser);
const PASSWORD = encodeURIComponent(dbPassword);
const URI = `postgres://${USER}:${PASSWORD}@${dbHost}:${dbPort}/${dbName}`;
console.log('my uri');
console.log(URI);

const sequelize = new Sequelize(URI, {
  dialect: 'postgres',
  logging: true,
});

try {
  await sequelize.authenticate();
  console.log('Connection has been established successfully.');
} catch (error) {
  console.error('Unable to connect to the database: ', error);
}

setupModels(sequelize);

try {
  await sequelize.sync({ force: false });
  console.log('All models were synchronized successfully.');
} catch (error) {
  console.error('Unable to create tables : ', error);
}

export default sequelize;
