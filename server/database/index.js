import { User, UserSchema } from './user.model';

function setupModels(sequelize) {
  User.init(UserSchema, User.config(sequelize));
}

export default setupModels;
