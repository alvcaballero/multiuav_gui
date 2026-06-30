import { User, UserSchema } from './user.model.js';
import { Device, DeviceSchema } from './device.model.js';
import { Mission, MissionSchema } from './mission.model.js';
import { MissionRoute, MissionRouteSchema } from './missionRoute.model.js';
import { File, FileSchema } from './file.model.js';
import { Event, EventSchema } from './event.model.js';
import { Geofence, GeofenceSchema } from './geofence.model.js';
import { Chat, ChatSchema } from './chat.model.js';
import { ChatMessage, ChatMessageSchema } from './chatMessage.model.js';
import { MissionPlan, MissionPlanSchema } from './missionPlan.model.js';

export function setupModels(sequelize) {
  User.init(UserSchema, User.config(sequelize));
  Device.init(DeviceSchema, Device.config(sequelize));
  MissionPlan.init(MissionPlanSchema, MissionPlan.config(sequelize));
  Mission.init(MissionSchema, Mission.config(sequelize));
  MissionRoute.init(MissionRouteSchema, MissionRoute.config(sequelize));
  File.init(FileSchema, File.config(sequelize));
  Event.init(EventSchema, Event.config(sequelize));
  Geofence.init(GeofenceSchema, Geofence.config(sequelize));
  Chat.init(ChatSchema, Chat.config(sequelize));
  ChatMessage.init(ChatMessageSchema, ChatMessage.config(sequelize));

  // Associations
  Mission.belongsTo(MissionPlan, { foreignKey: 'planId', as: 'plan' });
  MissionPlan.hasMany(Mission, { foreignKey: 'planId', as: 'missions' });
}
