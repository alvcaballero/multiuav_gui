import dotenv from 'dotenv';
import path from 'path';
import { fileURLToPath } from 'url';
import { logger } from '../common/logger.js';

dotenv.config();

const __dirname = path.dirname(fileURLToPath(import.meta.url));

export const env = process.env.NODE_ENV || 'dev';
export const port = Number(process.env.PORT) || 4000;
export const CorsEnable = process.env.CORS_ENABLE === 'true';
export const RosEnable =
  typeof process.env.ROS_CONNECTION === 'undefined' ? true : process.env.ROS_CONNECTION !== 'false';
export const FbEnable = typeof process.env.FB_CONNECTION === 'undefined' ? true : process.env.FB_CONNECTION !== 'false';
export const StreamServer = process.env.STREAM_SERVER === 'true'; //if use mediamtx server in local host
export const LocalGlyphs = process.env.LOCAL_GLYPHS === 'true'; // if local server have a glyphs server
export const NoElevation = process.env.NO_ELEVATION === 'true'; // if local server have a glyphs server
export const useExternalDb = process.env.DB === 'true';
export const dbType = process.env.DB_TYPE;
export const dbUser = process.env.DB_USER;
export const dbPassword = process.env.DB_PASSWORD;
export const dbHost = process.env.DB_HOST;
export const dbName = process.env.DB_NAME;
export const dbPort = process.env.DB_PORT;
export const planningServer = process.env.PLANNING_SERVER === 'true';
export const planningHost = process.env.PLANNING_HOST;
export const missionDataPath = process.env.MISSION_DATA_PATH ?? '../data/';
export const processThermalImg = process.env.PROCESS_THERMAL_IMG === 'true';
// Absolute path so the invocation does not depend on the process CWD.
// `uv run --project <dir>` resolves the pyproject.toml/.venv living next to the script.
const thermalProjectDir = path.resolve(__dirname, '../utils/proccessThermalImg');
const thermalScriptPath = path.join(thermalProjectDir, 'processThermalGen.py');
export const processThermalScript =
  process.env.PROCESS_THERMAL_IMG_SRC ?? `uv run --project "${thermalProjectDir}" "${thermalScriptPath}"`;
export const extApp = process.env.EXT_APP === 'true';
export const extAppUrl = process.env.EXT_APP_URL || '';
export const extAppUser = process.env.EXT_APP_USER || '';
export const extAppPWD = process.env.EXT_APP_PWD || '';
export const LLM = process.env.LLM === 'true';
export const LLMProvider = process.env.LLM_PROVIDER || 'openai';
export const LLMApiKeys = {
  openai: process.env.LLM_OPENAI_API_KEY || '',
  gemini: process.env.LLM_GEMINI_API_KEY || '',
  anthropic: process.env.LLM_ANTHROPIC_API_KEY || '',
  // Local servers authenticate nothing, but an entry must exist so the bootstrap's
  // "key required" check can be satisfied without special-casing each provider.
  ollama: process.env.LLM_OLLAMA_API_KEY || 'not-needed',
  llamacpp: process.env.LLM_LLAMACPP_API_KEY || 'not-needed',
  'openai-compatible': process.env.LLM_COMPATIBLE_API_KEY || 'not-needed',
};

/**
 * Chat Completions endpoints for OpenAI-compatible providers.
 * Empty means "use the preset default" from handlers/compatibleProviders.js.
 * The `/v1` suffix is appended automatically when missing.
 */
export const LLMBaseURLs = {
  // The endpoint lives here and nowhere else. `LLM_OLLAMA_API_KEY` used to carry the host
  // for the native client; a URL left in it now aborts startup (server.js) instead of being
  // silently ignored, which would strand a remote host on the localhost default.
  ollama: process.env.LLM_OLLAMA_BASE_URL || '',
  llamacpp: process.env.LLM_LLAMACPP_BASE_URL || '',
  'openai-compatible': process.env.LLM_COMPATIBLE_BASE_URL || '',
};

/** Optional model override. Empty means "use the provider's default". */
export const LLMModel = process.env.LLM_MODEL || '';
export const MCPenable = process.env.MCP_ENABLE === 'true'; // Model Context Protocol

// eve agent orchestrator (gcs_eevee_assistant) — opt-in per chat via metadata.engine === 'eve',
// runs alongside the legacy MessageOrchestrator/mcpClient path, does not replace it.
export const EveEnable = process.env.EVE_ENABLE === 'true';
export const EveUrl = process.env.EVE_URL || 'http://127.0.0.1:2000';

const VALID_MCP_TRANSPORTS = ['stdio', 'http', 'sse'];
let _MCPconfig = {};
const raw = process.env.MCP_CONFIG;
if (raw) {
  try {
    _MCPconfig = JSON.parse(raw);
    if (!VALID_MCP_TRANSPORTS.includes(_MCPconfig.transport)) {
      throw new Error(`Unknown transport '${_MCPconfig.transport}'. Valid: ${VALID_MCP_TRANSPORTS.join(', ')}`);
    }
    logger.info('MCP config loaded', _MCPconfig);
  } catch (err) {
    logger.error('Invalid MCP_CONFIG:', err.message);
    _MCPconfig = {};
  }
}
export const MCPconfig = _MCPconfig; // MCP configuration file

// Tools the chat agent may not execute without an explicit human approval.
// Default-deny would be safer but breaks every read-only tool, so this is an
// explicit list of the tools that move a real aircraft.
export const TOOL_APPROVAL_REQUIRED = (process.env.TOOL_APPROVAL_REQUIRED ?? 'load_mission_to_uav,start_mission')
  .split(',')
  .map((name) => name.trim())
  .filter(Boolean);

export const TOOL_APPROVAL_TTL_MS = Number(process.env.TOOL_APPROVAL_TTL_MS) || 300000;
export const TOOL_APPROVAL_SWEEP_INTERVAL_MS = Number(process.env.TOOL_APPROVAL_SWEEP_INTERVAL_MS) || 30000;

// ON by default, and opt-OUT rather than opt-in: a safety gate that a missing
// env var silently disables is not a safety gate. Set to 'false' explicitly to
// let the agent fly without asking — e.g. an automated simulation run.
export const TOOL_APPROVAL_ENFORCE = process.env.TOOL_APPROVAL_ENFORCE !== 'false';

export const missionsConfigData = '../data/missionConfig.yaml';
// intervals
export const mapLatitude = Number(process.env.MAP_LATITUDE) || 37.19384681403371;
export const mapLongitude = Number(process.env.MAP_LONGITUDE) || -6.702598762315071;
export const mapZoom = Number(process.env.MAP_ZOOM) || 15;
export const WS_PING_INTERVAL_MS = Number(process.env.WS_PING_INTERVAL_MS) || 30000;
export const WS_POSITIONS_INTERVAL_MS = Number(process.env.WS_POSITIONS_INTERVAL_MS) || 500; // POSITION_UPDATED batch flush tick (see positionBroadcastBatcher.js)
export const WS_STATE_INTERVAL_MS = Number(process.env.WS_STATE_INTERVAL_MS) || 10000;
export const ROS_URL = process.env.ROS_URL || 'ws://127.0.0.1:9090';
export const ROS_RECONNECT_INTERVAL_MS = Number(process.env.ROS_RECONNECT_INTERVAL_MS) || 30000;
export const DEVICE_CHECK_INTERVAL_MS = Number(process.env.DEVICE_CHECK_INTERVAL_MS) || 5000;
export const DEVICE_UPDATE_INTERVAL_MS = Number(process.env.DEVICE_UPDATE_INTERVAL_MS) || 2000;
export const DEVICE_TIMEOUT_MS = Number(process.env.DEVICE_TIMEOUT_MS) || 30000;
// config files
export const devicesMsg = '../config/devices/devices_msg.yaml';
export const missionSchema = '../config/devices/mission_schema.yaml';
export const messagesTypes = '../config/devices/messages.yaml';
