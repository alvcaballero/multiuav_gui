import path from 'path';
import fs from 'fs';
import { fileURLToPath } from 'url';
import { readDataFile, writeDataFile } from '../common/utils.js';
import { missionsConfigData } from '../config/config.js';
import { logger } from '../common/logger.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const STATIC_TYPES_PATH = '../config/planning/elementTypes.yaml';
const CUSTOM_TYPES_PATH = '../data/markerTypes.yaml';
const ASSETS_DIR = path.resolve(__dirname, '../data/element-types');

const staticTypes = readDataFile(STATIC_TYPES_PATH) || [];
var initPlanning = readDataFile(missionsConfigData);

function loadCustomTypes() {
  const data = readDataFile(CUSTOM_TYPES_PATH);
  return Array.isArray(data) ? data : [];
}

function saveCustomTypes(types) {
  return writeDataFile(CUSTOM_TYPES_PATH, types);
}

export class markersModel {
  // ─── Marker instances (bases + elements) ─────────────────────────────────

  static getMarkers() {
    logger.debug('get markers');
    return { markersbase: initPlanning.markersbase, elements: initPlanning.elements };
  }

  static setMarkers(value) {
    logger.debug('Set markers');
    const updated = { ...initPlanning, markersbase: value.markersbase, elements: value.elements };
    initPlanning = updated;
    return { result: writeDataFile(missionsConfigData, updated) };
  }

  static getBases() {
    logger.debug('get bases');
    return initPlanning.markersbase;
  }

  static getElements() {
    logger.debug('get elements');
    return initPlanning.elements;
  }

  static getBaseswithAssignments() {
    logger.debug('get bases with assignments');
    const assignments = initPlanning.assignments || [];
    const bases = initPlanning.markersbase || [];
    const assignmentsMap = new Map(assignments.map((a) => [a.baseId, a.device]));
    return bases.map((base) => ({ ...base, device: assignmentsMap.get(base.id) || null }));
  }

  // ─── Element type catalog ─────────────────────────────────────────────────

  static getAllTypes() {
    logger.debug('get all marker types');
    const custom = loadCustomTypes().map((t) => ({ ...t, custom: true }));
    return [...staticTypes.map((t) => ({ ...t, custom: false })), ...custom];
  }

  static getCustomTypes() {
    logger.debug('get custom marker types');
    return loadCustomTypes();
  }

  static createCustomType({ name, description, height, color }) {
    logger.debug(`create custom marker type: ${name}`);
    const customTypes = loadCustomTypes();
    const id = `custom_${Date.now()}`;

    const newType = {
      id,
      name,
      description: description || '',
      height: height || 10,
      color: color || 'gray',
      icon: null,
      model3d: null,
    };

    customTypes.push(newType);
    saveCustomTypes(customTypes);
    return newType;
  }

  static deleteCustomType(id) {
    logger.debug(`delete custom marker type: ${id}`);
    const customTypes = loadCustomTypes().filter((t) => t.id !== id);
    saveCustomTypes(customTypes);

    // Remove assets if they exist
    const assetDir = path.join(ASSETS_DIR, id);
    if (fs.existsSync(assetDir)) {
      fs.rmSync(assetDir, { recursive: true });
    }

    return { result: true };
  }

  static saveTypeIcon(id, _filePath) {
    logger.debug(`save icon for type: ${id}`);
    const customTypes = loadCustomTypes();
    const idx = customTypes.findIndex((t) => t.id === id);
    if (idx === -1) throw new Error(`Type ${id} not found`);

    customTypes[idx].icon = `/api/markers/types/${id}/icon`;
    saveCustomTypes(customTypes);
    return customTypes[idx];
  }

  static saveTypeModel(id, _filePath) {
    logger.debug(`save 3d model for type: ${id}`);
    const customTypes = loadCustomTypes();
    const idx = customTypes.findIndex((t) => t.id === id);
    if (idx === -1) throw new Error(`Type ${id} not found`);

    customTypes[idx].model3d = `/api/markers/types/${id}/model`;
    saveCustomTypes(customTypes);
    return customTypes[idx];
  }

  static getAssetPath(id, assetType) {
    const ext = assetType === 'icon' ? ['png', 'svg', 'jpg'] : ['glb', 'gltf'];
    const dir = path.join(ASSETS_DIR, id);
    if (!fs.existsSync(dir)) return null;

    for (const e of ext) {
      const p = path.join(dir, `${assetType}.${e}`);
      if (fs.existsSync(p)) return p;
    }
    return null;
  }

  static ensureAssetDir(id) {
    const dir = path.join(ASSETS_DIR, id);
    if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
    return dir;
  }
}
