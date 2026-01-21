/**
 * Índice central de todos los prompts del sistema
 * Carga los prompts desde archivos .md externos
 */

import { readFileSync } from 'fs';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

/**
 * Carga un prompt desde un archivo .md
 * @param {string} filename - Nombre del archivo .md
 * @returns {string} Contenido del prompt
 */
const loadPrompt = (filename) => readFileSync(join(__dirname, filename), 'utf8');

/**
 * Objeto con todos los prompts del sistema
 */
export const SystemPrompts = {
  main: loadPrompt('main-prompt.md'),
  agv: loadPrompt('agv-prompt.md'),
  other: loadPrompt('other-prompt.md'),
  mission_build_xyz: loadPrompt('mission-build-xyz.md'),
  verification_mission_briefing: loadPrompt('verification-mission-briefing.md'),
};

// Exportaciones individuales para compatibilidad
export const mainPrompt = SystemPrompts.main;
export const agvPrompt = SystemPrompts.agv;
export const otherPrompt = SystemPrompts.other;
export const missionBuildXyzPrompt = SystemPrompts.mission_build_xyz;
export const verificationMissionBriefingPrompt = SystemPrompts.verification_mission_briefing;
