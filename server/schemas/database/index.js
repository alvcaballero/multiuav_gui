import { User, UserSchema } from './user.model.js';
import { Device, DeviceSchema } from './device.model.js';
import { Mission, MissionSchema } from './mission.model.js';
import { Route, RouteSchema } from './routes.model.js';
import { File, FileSchema } from './file.model.js';
import { Event, EventSchema } from './event.model.js';
import { Geofence, GeofenceSchema } from './geofence.model.js';
import { Chat, ChatSchema } from './chat.model.js';
import { ChatMessage, ChatMessageSchema } from './chatMessage.model.js';

export function setupModels(sequelize) {
  User.init(UserSchema, User.config(sequelize));
  Device.init(DeviceSchema, Device.config(sequelize));
  Mission.init(MissionSchema, Mission.config(sequelize));
  Route.init(RouteSchema, Route.config(sequelize));
  File.init(FileSchema, File.config(sequelize));
  Event.init(EventSchema, Event.config(sequelize));
  Geofence.init(GeofenceSchema, Geofence.config(sequelize));
  Chat.init(ChatSchema, Chat.config(sequelize));
  ChatMessage.init(ChatMessageSchema, ChatMessage.config(sequelize));
}
