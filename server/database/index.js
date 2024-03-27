import { User, UserSchema } from './user.model.js';
import { Device, DeviceSchema } from './device.model.js';
import { Mission, MissionSchema } from './mission.model.js';
import { Route, RouteSchema } from './routes.model.js';
import { Media, MediaSchema } from './media.model.js';
import { Event, EventSchema } from './event.model.js';

export function setupModels(sequelize) {
  User.init(UserSchema, User.config(sequelize));
  Device.init(DeviceSchema, Device.config(sequelize));
  Mission.init(MissionSchema, Mission.config(sequelize));
  Route.init(RouteSchema, Route.config(sequelize));
  Media.init(MediaSchema, Media.config(sequelize));
  Event.init(EventSchema, Event.config(sequelize));
}
