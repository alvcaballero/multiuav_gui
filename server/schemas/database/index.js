import { User, UserSchema } from './user.model.js';
import { Device, DeviceSchema } from './device.model.js';
import { Mission, MissionSchema } from './mission.model.js';
import { MissionRoute, MissionRouteSchema } from './missionRoute.model.js';
import { File, FileSchema } from './file.model.js';
import { Event, EventSchema } from './event.model.js';
import { Geofence, GeofenceSchema } from './geofence.model.js';
import { Chat, ChatSchema } from './chat.model.js';
import { ChatMessage, ChatMessageSchema } from './chatMessage.model.js';
import { ChatUsage, ChatUsageSchema } from './chatUsage.model.js';
import { MissionPlan, MissionPlanSchema } from './missionPlan.model.js';
import { PositionHistory, PositionHistorySchema } from './positionHistory.model.js';
import { ElementType, ElementTypeSchema } from './elementType.model.js';
import { ElementGroup, ElementGroupSchema } from './elementGroup.model.js';
import { ElementItem, ElementItemSchema } from './elementItem.model.js';
import { Base, BaseSchema } from './base.model.js';
import { Assignment, AssignmentSchema } from './assignment.model.js';

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
  ChatUsage.init(ChatUsageSchema, ChatUsage.config(sequelize));
  PositionHistory.init(PositionHistorySchema, PositionHistory.config(sequelize));
  ElementType.init(ElementTypeSchema, ElementType.config(sequelize));
  ElementGroup.init(ElementGroupSchema, ElementGroup.config(sequelize));
  ElementItem.init(ElementItemSchema, ElementItem.config(sequelize));
  Base.init(BaseSchema, Base.config(sequelize));
  Assignment.init(AssignmentSchema, Assignment.config(sequelize));

  // Associations
  Mission.belongsTo(MissionPlan, { foreignKey: 'planId', as: 'plan' });
  MissionPlan.hasMany(Mission, { foreignKey: 'planId', as: 'missions' });
  PositionHistory.belongsTo(Device, { foreignKey: 'deviceId' });
  ElementGroup.belongsTo(ElementType, { foreignKey: 'typeId', as: 'type' });
  ElementItem.belongsTo(ElementGroup, { foreignKey: 'groupId', as: 'group' });
  ElementGroup.hasMany(ElementItem, { foreignKey: 'groupId', as: 'items' });
  Base.belongsTo(ElementType, { foreignKey: 'typeId', as: 'type' });
  Assignment.belongsTo(Base, { foreignKey: 'baseId', as: 'base' });
  Assignment.belongsTo(Device, { foreignKey: 'deviceId', as: 'device' });
}
