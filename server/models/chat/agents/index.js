import { readFileSync } from 'fs';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';

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

// Named exports for individual access
export const defaultAgent = agents.default;
export const plannerAgent = agents.planner;
export const agvAgent = agents.agv;
export const otherAgent = agents.other;
export const verificationMissionAgent = agents['verification-mission'];

// Legacy compatibility — same shape as old SystemPrompts
export const SystemPrompts = {
  main: agents.default.systemPrompt,
  agv: agents.agv.systemPrompt,
  other: agents.other.systemPrompt,
  mission_build_xyz: agents.planner.systemPrompt,
  verification_mission: agents['verification-mission'].systemPrompt,
};
