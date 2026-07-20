import sequelize, { Op } from '../../common/sequelize.js';
import { logger } from '../../common/logger.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';

/**
 * Persistencia del histórico de posiciones/telemetría.
 *
 * - `record()` inserta una fila (best-effort: nunca debe tumbar al sampler).
 * - Retención FIFO **throttled**: no se cuenta ni se poda en cada inserción, sino
 *   cada RETENTION_CHECK_EVERY inserts, para no castigar el pool `max:1` de SQLite.
 * - Al alcanzar el 90% del cupo se emite un warning (log + EventBus) una sola vez
 *   por cruce de umbral.
 */

// Cupo total de la tabla (global, compartido por todos los dispositivos).
// ~300-450 bytes/fila ⇒ 500k filas ≈ 150-220 MB en SQLite.
const MAX_ROWS = 500_000;
// Umbral de aviso (90% del cupo).
const WARN_ROWS = Math.floor(MAX_ROWS * 0.9);
// Cada cuántos inserts se corre el COUNT + poda (throttling).
const RETENTION_CHECK_EVERY = 500;
// Tope de filas devueltas por una consulta histórica (evita volcar toda la tabla
// en un request). Si se alcanza, la respuesta marca `truncated: true`.
const DEFAULT_QUERY_LIMIT = 10_000;

export class PositionHistoryModel {
  // Contador de inserts desde la última verificación de retención.
  static _insertsSinceCheck = 0;
  // Evita spamear el warning: solo se avisa al cruzar el umbral hacia arriba.
  static _warned = false;

  /**
   * Con `raw: true`, Sequelize NO deserializa DataTypes.JSON en SQLite: `attributes`
   * vuelve como string. Normaliza cada fila a un objeto `attributes` parseado.
   */
  static _parseRow(row) {
    if (!row) return row;
    if (typeof row.attributes === 'string') {
      try {
        row.attributes = JSON.parse(row.attributes);
      } catch {
        row.attributes = {};
      }
    }
    return row;
  }

  /**
   * Inserta una fila del histórico. Best-effort: cualquier error se loguea y se
   * traga para no interrumpir el ciclo del sampler.
   * @param {object} row - { deviceId, fixTime, latitude, longitude, altitude, course, speed, attributes }
   * @returns {Promise<boolean>} true si se insertó, false si falló.
   */
  static async record(row) {
    try {
      await sequelize.models.PositionHistory.create({
        deviceId: row.deviceId,
        fixTime: row.fixTime ?? new Date(),
        deviceTime: row.deviceTime ?? null,
        latitude: row.latitude ?? null,
        longitude: row.longitude ?? null,
        altitude: row.altitude ?? null,
        course: row.course ?? null,
        speed: row.speed ?? null,
        attributes: row.attributes ?? {},
      });

      this._insertsSinceCheck += 1;
      if (this._insertsSinceCheck >= RETENTION_CHECK_EVERY) {
        this._insertsSinceCheck = 0;
        // No await: la poda corre en background, no bloquea al sampler.
        this.enforceRetention().catch((err) =>
          logger.error(`PositionHistory retention error: ${err.message}`)
        );
      }
      return true;
    } catch (err) {
      logger.error(`PositionHistory.record failed device=${row.deviceId}: ${err.message}`);
      return false;
    }
  }

  /**
   * FIFO: si la tabla supera MAX_ROWS, borra las filas más viejas hasta volver al
   * límite. Emite warning al cruzar WARN_ROWS.
   */
  static async enforceRetention() {
    const Model = sequelize.models.PositionHistory;
    const count = await Model.count();

    // Aviso de 90% (una sola vez por cruce hacia arriba).
    if (count >= WARN_ROWS && !this._warned) {
      this._warned = true;
      const msg = `PositionHistory alcanzó ${count}/${MAX_ROWS} filas (>=90%). Se podará por FIFO.`;
      logger.warn(msg);
      eventBus.emitSafe(EVENTS.POSITION_HISTORY_WARNING, { count, max: MAX_ROWS, threshold: WARN_ROWS });
    } else if (count < WARN_ROWS) {
      // Rearmar el aviso cuando bajamos del umbral.
      this._warned = false;
    }

    if (count <= MAX_ROWS) return;

    const toDelete = count - MAX_ROWS;
    // Borra las `toDelete` filas más viejas (por id ascendente = orden de inserción).
    const oldest = await Model.findAll({
      attributes: ['id'],
      order: [['id', 'ASC']],
      limit: toDelete,
      raw: true,
    });
    if (oldest.length === 0) return;
    const ids = oldest.map((r) => r.id);
    const deleted = await Model.destroy({ where: { id: ids } });
    logger.info(`PositionHistory FIFO: borradas ${deleted} filas (${count} → ${count - deleted}).`);
  }

  /**
   * Consulta la traza de un dispositivo (para futura UI/export).
   * @param {number} deviceId
   * @param {{ from?: Date, to?: Date, limit?: number }} [opts]
   */
  static async getByDevice(deviceId, { from, to, limit = 1000 } = {}) {
    const where = { deviceId };
    if (from || to) {
      where.fixTime = {};
      if (from) where.fixTime[Op.gte] = from;
      if (to) where.fixTime[Op.lte] = to;
    }
    return sequelize.models.PositionHistory.findAll({
      where,
      order: [['fixTime', 'ASC']],
      limit,
    });
  }

  /**
   * Consulta histórica por rango de tiempo, opcionalmente filtrada por device.
   * Pensada para el endpoint GET /positions con from/to.
   *
   * @param {{ deviceId?: number, from: Date, to: Date, limit?: number }} opts
   * @returns {Promise<{ rows: object[], truncated: boolean }>} filas ordenadas por
   *   fixTime ASC; `truncated` avisa si se alcanzó el límite (hay más datos).
   */
  static async getHistory({ deviceId, from, to, limit = DEFAULT_QUERY_LIMIT } = {}) {
    const where = {};
    if (deviceId !== undefined && deviceId !== null) where.deviceId = deviceId;
    if (from || to) {
      where.fixTime = {};
      if (from) where.fixTime[Op.gte] = from;
      if (to) where.fixTime[Op.lte] = to;
    }
    // Pedimos limit+1 para saber si había más filas de las devueltas (truncado).
    const rows = await sequelize.models.PositionHistory.findAll({
      where,
      order: [['fixTime', 'ASC']],
      limit: limit + 1,
      raw: true,
    });
    const truncated = rows.length > limit;
    const sliced = truncated ? rows.slice(0, limit) : rows;
    return { rows: sliced.map((r) => this._parseRow(r)), truncated };
  }

  /**
   * Última fila (por fixTime) de cada dispositivo. Usada para precargar la caché
   * en RAM al arrancar el server.
   *
   * @param {number[]} [deviceIds] - si se pasa, restringe a esos devices.
   * @returns {Promise<object[]>} una fila por device (la más reciente).
   */
  static async getLatestPerDevice(deviceIds) {
    const Model = sequelize.models.PositionHistory;
    const scoped = Array.isArray(deviceIds) && deviceIds.length > 0;

    // La última fila de cada device es la de mayor `id` (autoincremental) para ese
    // deviceId. Filtramos por MAX(id) POR deviceId con una subconsulta: comparar
    // enteros evita pasar el timestamp de vuelta por Sequelize/moment (que dispara
    // un deprecation warning con el formato DATETIME crudo de SQLite).
    const rows = await Model.findAll({
      where: {
        ...(scoped ? { deviceId: { [Op.in]: deviceIds } } : {}),
        id: {
          [Op.in]: sequelize.literal('(SELECT MAX(id) FROM PositionHistory GROUP BY deviceId)'),
        },
      },
      raw: true,
    });
    return rows.map((r) => this._parseRow(r));
  }
}

export const positionHistoryConfig = { MAX_ROWS, WARN_ROWS, RETENTION_CHECK_EVERY, DEFAULT_QUERY_LIMIT };
