import sequelize from '../common/sequelize.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import logger from '../common/logger.js';
import { ROUTE_STATUS, MISSION_STATUS } from '../config/status.js';

const WP_REACHED_THRESHOLD_M = 2;

// TODO:
//  - error cases like timeouts, device landing , exit of mission
//

// Keyed by deviceId → { planId, missionData, loadedAt }
// Allows multiple operators loading different missions for different UAVs concurrently.
const _pendingByDevice = {};

function haversineMeters(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const toRad = (d) => (d * Math.PI) / 180;
  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);
  const a = Math.sin(dLat / 2) ** 2 + Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

export class missionWpTracking {
  /**
   * Called by commandsModel.loadmissionDevice after the ROS/FB command succeeds.
   * Stores plan metadata per device so commandMission can pick it up later.
   */
  static async onMissionLoaded(deviceIds, missionData) {
    const { missionModel } = await import('./mission.js');
    const plan = await missionModel.createMissionPlan(missionData, { source: 'manual' });
    logger.info(`WpTracking: MissionPlan created id=${plan.id} for devices=[${deviceIds.join(',')}]`);

    for (const deviceId of deviceIds) {
      _pendingByDevice[deviceId] = { planId: plan.id, missionData, loadedAt: Date.now() };
      logger.debug(`WpTracking: pending registered device=${deviceId} planId=${plan.id}`);
    }
  }

  /**
   * Called by commandsModel.commandMissionDevice after the ROS/FB command succeeds.
   * Creates Mission + MissionRoute records for each commanded device using their pending plan.
   */
  static async onMissionCommanded(commandedDeviceIds) {
    const { devicesController } = await import('../controllers/devices.js');
    const { missionModel } = await import('./mission.js');

    // Group commanded devices by planId — same plan → one Mission record
    const planGroups = {};
    for (const devId of commandedDeviceIds) {
      const pending = _pendingByDevice[devId];
      if (!pending) {
        logger.warn(`WpTracking: no pending plan for device=${devId}, skipping MissionRoute creation`);
        continue;
      }
      const key = pending.planId;
      if (!planGroups[key]) {
        planGroups[key] = { planId: pending.planId, missionData: pending.missionData, deviceIds: [] };
      }
      planGroups[key].deviceIds.push(devId);
    }

    for (const group of Object.values(planGroups)) {
      try {
        const mission = await missionModel.createMission({
          name: `manual_${group.planId}`,
          planId: group.planId,
          trigger: 'manual',
          uav: group.deviceIds,
          status: MISSION_STATUS.RUNNING,
          mission: group.missionData,
        });
        logger.info(
          `WpTracking: Mission created id=${mission.id} planId=${group.planId} devices=[${group.deviceIds.join(',')}]`
        );

        for (const devId of group.deviceIds) {
          const devRoute = await this._findRouteForDevice(group.missionData, devId, devicesController);
          const totalWp = devRoute?.wp?.length ?? 0;
          await missionModel.createRoute({
            missionId: mission.id,
            deviceId: devId,
            status: ROUTE_STATUS.COMMANDED,
            initTime: new Date(),
            currentWp: 0,
            totalWp,
          });
          logger.debug(`WpTracking: MissionRoute created device=${devId} totalWp=${totalWp}`);
          delete _pendingByDevice[devId];
        }
      } catch (err) {
        logger.error(`WpTracking: error creating Mission for planId=${group.planId}: ${err.message}`);
      }
    }
  }

  /**
   * Called on every position update that has lat/lon.
   * Checks if the device is running an active MissionRoute and advances WP counter.
   */
  static async checkProgress(deviceId, position) {
    if (!position?.latitude || !position?.longitude) return;

    let missionRoute;
    try {
      missionRoute = await sequelize.models.MissionRoute.findOne({
        where: { deviceId, status: [ROUTE_STATUS.COMMANDED, ROUTE_STATUS.RUNNING] },
      });
    } catch (err) {
      logger.error(`WpTracking: DB error device=${deviceId}: ${err.message}`);
      return;
    }

    if (!missionRoute) return;

    const mission = await sequelize.models.Mission.findOne({ where: { id: missionRoute.missionId } });
    if (!mission?.planId) return;

    const plan = await sequelize.models.MissionPlan.findOne({ where: { id: mission.planId } });
    if (!plan?.missionData?.route) return;

    const { devicesController } = await import('../controllers/devices.js');
    const devRoute = await this._findRouteForDevice(plan.missionData, deviceId, devicesController);
    if (!devRoute?.wp?.length) return;

    const waypoints = devRoute.wp;
    const currentWp = missionRoute.currentWp ?? 0;
    if (currentWp >= waypoints.length) return;

    const target = waypoints[currentWp];
    const [tLat, tLon] = Array.isArray(target.pos) ? target.pos : [target.pos.lat, target.pos.lon];
    const dist = haversineMeters(position.latitude, position.longitude, tLat, tLon);

    logger.debug(
      `WpTracking device=${deviceId} route=${missionRoute.id} wp=${currentWp}/${waypoints.length} dist=${dist.toFixed(1)}m`
    );

    if (dist > WP_REACHED_THRESHOLD_M) return;

    const nextWp = currentWp + 1;
    const isLast = nextWp >= waypoints.length;

    missionRoute.currentWp = nextWp;
    missionRoute.status = isLast ? ROUTE_STATUS.COMPLETED : ROUTE_STATUS.RUNNING;
    if (isLast) missionRoute.endTime = new Date();
    await missionRoute.save();

    logger.info(`WpTracking device=${deviceId} wp reached ${currentWp} → next=${nextWp} last=${isLast}`);

    eventBus.emitSafe(EVENTS.MISSION_PROGRESS, {
      missionId: missionRoute.missionId,
      deviceId,
      routeId: missionRoute.id,
      currentWp: nextWp,
      totalWp: missionRoute.totalWp,
      completed: isLast,
    });

    if (isLast) await this._checkMissionComplete(missionRoute.missionId);
  }

  static async _findRouteForDevice(missionData, deviceId, devicesController) {
    for (const r of missionData.route ?? []) {
      const device = await devicesController.getByName(r.uav);
      if (device?.id === deviceId) return r;
    }
    return null;
  }

  static async _checkMissionComplete(missionId) {
    const routes = await sequelize.models.MissionRoute.findAll({ where: { missionId } });
    if (!routes.every((r) => r.status === ROUTE_STATUS.COMPLETED)) return;

    await sequelize.models.Mission.update(
      { status: MISSION_STATUS.COMPLETED, endTime: new Date() },
      { where: { id: missionId } }
    );
    logger.info(`WpTracking: Mission ${missionId} all routes completed`);
    eventBus.emitSafe(EVENTS.MISSION_COMPLETED, { missionId });
  }
}
