import { readFileSync } from 'fs';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';
import sequelize from '../../../common/sequelize.js';
import { chatLogger } from '../../../common/logger.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

/**
 * Parses YAML frontmatter from a markdown file.
 * Supports string, null, and array values only — no nested objects.
 * @param {string} raw - Raw file content
 * @returns {{ meta: Object, body: string }}
 */
function parseFrontmatter(raw) {
  const match = raw.match(/^---\r?\n([\s\S]*?)\r?\n---\r?\n([\s\S]*)$/);
  if (!match) return { meta: {}, body: raw };

  const [, frontmatter, body] = match;
  const meta = {};

  let currentKey = null;
  for (const line of frontmatter.split('\n')) {
    const arrayItem = line.match(/^\s{2}-\s+(.+)$/);
    const nullValue = line.match(/^(\w[\w-]*):\s*null\s*$/);
    const keyValue = line.match(/^(\w[\w-]*):\s*(.+)$/);
    const arrayStart = line.match(/^(\w[\w-]*):\s*$/);

    if (nullValue) {
      meta[nullValue[1]] = null;
      currentKey = null;
    } else if (arrayStart) {
      currentKey = arrayStart[1];
      meta[currentKey] = [];
    } else if (arrayItem && currentKey) {
      meta[currentKey].push(arrayItem[1].trim());
    } else if (keyValue) {
      meta[keyValue[1]] = keyValue[2].trim();
      currentKey = null;
    }
  }

  return { meta, body: body.trim() };
}

/**
 * Loads an agent definition from a .md file.
 * @param {string} filename
 * @returns {{ name: string, description: string, systemPrompt: string, allowedTools: string[]|null }}
 */
function loadAgent(filename) {
  const raw = readFileSync(join(__dirname, filename), 'utf8');
  const { meta, body } = parseFrontmatter(raw);
  return {
    name: meta.name ?? filename.replace('.md', ''),
    description: meta.description ?? '',
    capability: meta.capability ?? 'low',
    systemPrompt: body,
    allowedTools: meta.allowedTools ?? null,
  };
}

const agents = {
  default: loadAgent('default.md'),
  planner: loadAgent('planner.md'),
  agv: loadAgent('agv.md'),
  other: loadAgent('other.md'),
  'verification-mission': loadAgent('verification-mission.md'),
};

export { agents };

export const DEFAULT_AGENT = 'default';

/**
 * Resolves the full agent definition for a chat from DB metadata.
 * Falls back to DEFAULT_AGENT if not set or unknown.
 * @param {string} chatId
 * @returns {Promise<{ name: string, description: string, systemPrompt: string, allowedTools: string[]|null }>}
 */
export async function resolveAgentForChat(chatId) {
  try {
    const chat = await sequelize.models.Chat.findByPk(chatId);
    const profileName = chat?.metadata?.agentProfile ?? DEFAULT_AGENT;
    return resolveAgent(profileName);
  } catch (error) {
    chatLogger.error(`resolveAgentForChat: error for chat ${chatId}:`, error);
    return agents[DEFAULT_AGENT];
  }
}

/**
 * Resolves an agent by name. Falls back to DEFAULT_AGENT if unknown.
 * @param {string} name
 * @returns {{ name: string, description: string, systemPrompt: string, allowedTools: string[]|null }}
 */
export function resolveAgent(name) {
  if (!agents[name]) {
    chatLogger.warn(`resolveAgent: unknown profile '${name}', falling back to '${DEFAULT_AGENT}'`);
    return agents[DEFAULT_AGENT];
  }
  return agents[name];
}

/**
 * Persists the agent choice for a chat in DB metadata.
 * @param {string} chatId
 * @param {string} name
 */
export async function setAgentForChat(chatId, name) {
  if (!agents[name]) throw new Error(`setAgentForChat: unknown agent '${name}'`);
  try {
    const chat = await sequelize.models.Chat.findByPk(chatId);
    if (chat) {
      chat.metadata = { ...(chat.metadata || {}), agentProfile: name };
      chat.changed('metadata', true);
      chat.updatedAt = new Date();
      await chat.save();
      chatLogger.info(`setAgentForChat: agent '${name}' set for chat ${chatId}`);
    }
  } catch (error) {
    chatLogger.error(`setAgentForChat: error for chat ${chatId}:`, error);
  }
}

// Legacy compatibility — same shape as old SystemPrompts
export const SystemPrompts = {
  main: agents.default.systemPrompt,
  agv: agents.agv.systemPrompt,
  other: agents.other.systemPrompt,
  mission_build_xyz: agents.planner.systemPrompt,
  verification_mission: agents['verification-mission'].systemPrompt,
};
