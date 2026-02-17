#!/usr/bin/env node

/**
 * E2E Test: LLM Collision Avoidance with MCP + DB Persistence
 *
 * Tests the LLM's ability to detect and resolve collisions in a UAV mission
 * using the real MCP server (validate_mission_collisions) and persisting
 * everything in the real SQLite DB.
 *
 * Follows the subAgentPlannerChat pattern from chat.js.
 *
 * Prerequisites:
 * - MCP server running: cd mcp_server && npm run dev:http
 * - OpenAI API key in .env (LLM_OPENAI_API_KEY)
 * - SQLite DB accessible (data/sequelize.sqlite)
 *
 * Run: cd multiuav_gcs/server && node test/test-llm-collision-avoidance.js
 */

import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

// ═══════════════════════════════════════════════════════════════════
// STEP 1: Override env vars BEFORE importing modules (ESM evaluation order)
// ═══════════════════════════════════════════════════════════════════
process.env.MCP_ENABLE = 'true';
process.env.MCP_CONFIG = JSON.stringify({
  transport: 'http',
  url: 'http://127.0.0.1:3001/mcp/',
});
process.env.LLM = 'true';

// Dynamic imports (config.js reads env vars at import time)
const { MCPclient } = await import('../models/chat/mcpClient.js');
const { LLMFactory } = await import('../models/chat/llmFactory.js');
const { ChatHistoryManager } = await import('../models/chat/chatHistoryManager.js');
const sequelize = (await import('../common/sequelize.js')).default;
const { encode } = await import('@toon-format/toon');
const { decode } = await import('@toon-format/toon');
const { LLMApiKeys, LLMProvider } = await import('../config/config.js');

// ═══════════════════════════════════════════════════════════════════
// Paths & fixtures
// ═══════════════════════════════════════════════════════════════════
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const jsonDir = join(__dirname, 'api', 'json');
const promptsDir = join(__dirname, '..', 'models', 'chat', 'prompts');

const collisionFixture = JSON.parse(readFileSync(join(jsonDir, 'collision_validate.json'), 'utf-8'));
const verificationPromptBase = readFileSync(join(promptsDir, 'verification-mission.md'), 'utf-8');

// ═══════════════════════════════════════════════════════════════════
// Test state
// ═══════════════════════════════════════════════════════════════════
let passed = 0;
let failed = 0;
const MAX_ITERATIONS = 6;

function assert(testName, condition, detail = '') {
  if (condition) {
    console.log(`  PASSED: ${testName}${detail ? ' — ' + detail : ''}`);
    passed++;
  } else {
    console.log(`  FAILED: ${testName}${detail ? ' — ' + detail : ''}`);
    failed++;
  }
}

// Track data across the loop for assertions
const toolCallLog = [];       // { name, args, result }
let firstValidationResult = null;
let lastValidationResult = null;
let finalAssistantText = '';

async function runTest() {
  console.log('='.repeat(60));
  console.log('E2E Test: LLM Collision Avoidance + MCP + DB Persistence');
  console.log('='.repeat(60));
  console.log();

  // ═══════════════════════════════════════════════════════════════
  // STEP 2: Initialize services
  // ═══════════════════════════════════════════════════════════════
  console.log('--- Setup ---');

  // MCP client
  const mcpClient = new MCPclient();
  console.log('Connecting to MCP server...');
  try {
    await mcpClient.connect();
    console.log('  MCP connected');
  } catch (error) {
    console.log(`  FATAL: MCP connection failed: ${error.message}`);
    console.log('  Make sure MCP server is running: cd mcp_server && npm run dev:http');
    process.exit(1);
  }

  // LLM handler
  const provider = LLMProvider || 'openai';
  const apiKey = LLMApiKeys[provider];
  if (!apiKey) {
    console.log(`  FATAL: No API key for provider "${provider}". Set LLM_OPENAI_API_KEY in .env`);
    process.exit(1);
  }
  const llmHandler = LLMFactory.createHandler(provider, apiKey);
  await llmHandler.initialize();
  console.log(`  LLM handler initialized (${provider})`);

  // ═══════════════════════════════════════════════════════════════
  // STEP 3: Create dedicated chat in DB
  // ═══════════════════════════════════════════════════════════════
  console.log('\n--- Creating chat ---');
  const chat = await ChatHistoryManager.createChat('test-collision-avoidance');
  const chatId = chat.id;
  console.log(`  Chat created: ${chatId}`);

  await ChatHistoryManager.setAllowedTools(chatId, ['validate_mission_collisions']);
  await ChatHistoryManager.setAgentProfile(chatId, 'planner');
  console.log('  Metadata set: agentProfile=planner, allowedTools=[validate_mission_collisions]');

  // ═══════════════════════════════════════════════════════════════
  // STEP 4: System prompt
  // ═══════════════════════════════════════════════════════════════
  const systemPromptContent = `${verificationPromptBase}
---
Session context:
- chat_id: ${chatId}
- coordinate_system: local XYZ (ENU - East/North/Up in meters)
- IMPORTANT: You have ONE tool available: validate_mission_collisions
- After detecting collisions, modify the waypoints to avoid obstacles and re-validate.
- When the mission is valid (valid: true), respond with the corrected mission JSON.`;

  await ChatHistoryManager.addMessage(chatId, 'system', {
    role: 'system',
    content: systemPromptContent,
  });
  console.log('  System prompt persisted');

  // ═══════════════════════════════════════════════════════════════
  // STEP 5: Build user message with TOON-encoded mission data
  // ═══════════════════════════════════════════════════════════════
  const userMessage = `Validate this mission against the obstacles and fix any collisions found.

## Mission Data
${encode(collisionFixture.mission)}

## Collision Objects (Obstacles)
${encode(collisionFixture.collision_objects)}

Use the validate_mission_collisions tool with the mission and collision_objects data. If collisions are detected, modify waypoints to create safe detours around obstacles and re-validate until the mission is safe.`;

  await ChatHistoryManager.addMessage(chatId, 'user', {
    role: 'user',
    content: userMessage,
  });
  console.log('  User message persisted');

  // ═══════════════════════════════════════════════════════════════
  // STEP 6: Get filtered tools (only validate_mission_collisions)
  // ═══════════════════════════════════════════════════════════════
  const allTools = mcpClient.getTools();
  const filteredTools = allTools.filter((t) => t.name === 'validate_mission_collisions');
  console.log(`  Tools available: ${filteredTools.map((t) => t.name).join(', ')} (filtered from ${allTools.length})`);

  if (filteredTools.length === 0) {
    console.log('  FATAL: validate_mission_collisions tool not found in MCP server');
    await mcpClient.disconnect();
    process.exit(1);
  }

  // ═══════════════════════════════════════════════════════════════
  // STEP 7: LLM + Tool call loop (manual, following chat.js pattern)
  // ═══════════════════════════════════════════════════════════════
  console.log('\n--- LLM Processing Loop ---');

  const persistence = ChatHistoryManager.getSessionPersistence();
  const sessionId = await llmHandler.ensureSession(chatId, persistence);
  console.log(`  Session: ${sessionId ? sessionId.substring(0, 25) + '...' : 'fallback (no session)'}`);

  // Load history for context
  let historySnapshot = await ChatHistoryManager.loadHistory(chatId);

  // First LLM call with user message
  let result = await llmHandler.processMessage(userMessage, filteredTools, historySnapshot, {
    sessionId,
    instructions: systemPromptContent,
    agentProfile: 'planner',
  });

  // Persist assistant output
  for (const res of result.output) {
    await ChatHistoryManager.addMessage(chatId, 'assistant', res, result.responseId);
  }

  let iterations = 0;
  let currentOutput = result.output;
  let currentResponseId = result.responseId;

  while (iterations < MAX_ITERATIONS) {
    iterations++;

    // Check if there are tool calls in the output
    const toolCalls = currentOutput.filter(
      (item) => item.type === 'function_call' || item.type === 'tool_call'
    );

    if (toolCalls.length === 0) {
      // No tool calls — LLM gave a text response, we're done
      console.log(`  Iteration ${iterations}: LLM responded with text (loop complete)`);
      break;
    }

    console.log(`  Iteration ${iterations}: ${toolCalls.length} tool call(s)`);

    // Execute each tool call
    const toolResults = [];
    for (const toolCall of toolCalls) {
      const toolName = toolCall.name;
      const toolArgs = JSON.parse(toolCall.arguments);
      console.log(`    → Executing: ${toolName}`);

      const mcpResult = await mcpClient.executeTool(toolName, toolArgs);

      // Parse the result for logging and assertions
      let parsedResult = null;
      try {
        const resultText = mcpResult.content?.[0]?.text || '';
        parsedResult = decode(resultText);
        console.log(
          `    ← Result: valid=${parsedResult.valid}, collisions=${parsedResult.totalCollisions}, warnings=${parsedResult.totalWarnings}`
        );
      } catch {
        console.log(`    ← Result: (raw, could not decode)`);
      }

      // Track for assertions
      toolCallLog.push({ name: toolName, args: toolArgs, result: parsedResult });
      if (!firstValidationResult) firstValidationResult = parsedResult;
      lastValidationResult = parsedResult;

      // Build function_call_output (same format as openaiHandler.handleToolCall)
      const toolOutput = {
        type: 'function_call_output',
        call_id: toolCall.call_id,
        name: toolName,
        output: JSON.stringify(mcpResult),
      };
      toolResults.push(toolOutput);

      // Persist tool result in DB
      await ChatHistoryManager.addMessage(chatId, 'assistant', toolOutput);
    }

    // Check if this is the last iteration — force text response
    const isLastIteration = iterations >= MAX_ITERATIONS;

    // Continue conversation with tool results
    const continueHistory = sessionId ? [] : await ChatHistoryManager.loadHistory(chatId);
    const continueTools = isLastIteration ? [] : filteredTools;

    result = await llmHandler.processMessage(null, continueTools, continueHistory, {
      sessionId,
      instructions: systemPromptContent,
      toolOutputs: toolResults,
      forceFinish: isLastIteration,
      agentProfile: 'planner',
    });

    currentOutput = result.output;
    currentResponseId = result.responseId;

    // Persist assistant output
    for (const res of currentOutput) {
      await ChatHistoryManager.addMessage(chatId, 'assistant', res, currentResponseId);
    }

    if (isLastIteration) {
      console.log('  Max iterations reached — forced final response');
    }
  }

  // Extract final assistant text
  for (const item of currentOutput) {
    if (item.type === 'text' || item.type === 'message') {
      if (typeof item.text === 'string') finalAssistantText += item.text;
      else if (typeof item.content === 'string') finalAssistantText += item.content;
      else if (Array.isArray(item.content)) {
        for (const block of item.content) {
          if (block.type === 'output_text' || block.type === 'text') {
            finalAssistantText += block.text || '';
          }
        }
      }
    }
  }

  console.log(`\n  Loop finished after ${iterations} iteration(s), ${toolCallLog.length} tool call(s)`);
  console.log(`  Final response preview: "${finalAssistantText.substring(0, 120)}..."`);

  // ═══════════════════════════════════════════════════════════════
  // STEP 8: Assertions
  // ═══════════════════════════════════════════════════════════════
  console.log('\n--- Assertions ---');

  // Test A: At least 1 tool call to validate_mission_collisions
  const collisionToolCalls = toolCallLog.filter((tc) => tc.name === 'validate_mission_collisions');
  assert(
    'Test A: LLM called validate_mission_collisions at least once',
    collisionToolCalls.length >= 1,
    `${collisionToolCalls.length} call(s)`
  );

  // Test B: First validation detected collisions
  assert(
    'Test B: First validation detected collisions',
    firstValidationResult && (firstValidationResult.valid === false || firstValidationResult.totalCollisions > 0),
    firstValidationResult
      ? `valid=${firstValidationResult.valid}, totalCollisions=${firstValidationResult.totalCollisions}`
      : 'no result'
  );

  // Test C: LLM generated a corrected mission in its response
  const hasMissionInResponse =
    finalAssistantText.includes('"wp"') ||
    finalAssistantText.includes('"route"') ||
    finalAssistantText.includes('waypoint') ||
    finalAssistantText.includes('corrected') ||
    finalAssistantText.includes('detour') ||
    finalAssistantText.includes('DETOUR');
  assert(
    'Test C: LLM response contains corrected mission or detour references',
    hasMissionInResponse,
    `response length: ${finalAssistantText.length} chars`
  );

  // Test D: If re-validation happened, check improvement
  if (collisionToolCalls.length >= 2) {
    const lastResult = collisionToolCalls[collisionToolCalls.length - 1].result;
    const improved =
      lastResult &&
      (lastResult.valid === true || lastResult.totalCollisions < firstValidationResult.totalCollisions);
    assert(
      'Test D: Re-validation shows improvement (valid or fewer collisions)',
      improved,
      lastResult
        ? `valid=${lastResult.valid}, totalCollisions=${lastResult.totalCollisions}`
        : 'no result'
    );
  } else {
    console.log('  SKIPPED: Test D (no re-validation — only 1 tool call)');
  }

  // Test E: Messages persisted in DB
  const history = await ChatHistoryManager.loadHistory(chatId);
  assert(
    'Test E: Messages persisted in DB',
    history.length > 0,
    `${history.length} messages`
  );

  // Test F: Chat metadata has correct agentProfile and allowedTools
  const storedProfile = await ChatHistoryManager.getAgentProfile(chatId);
  const storedTools = await ChatHistoryManager.getAllowedTools(chatId);
  assert(
    'Test F: Chat metadata — agentProfile=planner',
    storedProfile === 'planner',
    `got: ${storedProfile}`
  );
  assert(
    'Test F: Chat metadata — allowedTools includes validate_mission_collisions',
    Array.isArray(storedTools) && storedTools.includes('validate_mission_collisions'),
    `got: ${JSON.stringify(storedTools)}`
  );

  // ═══════════════════════════════════════════════════════════════
  // STEP 9: Cleanup
  // ═══════════════════════════════════════════════════════════════
  console.log('\n--- Cleanup ---');
  console.log(`  Chat "${chatId}" preserved in DB for manual inspection.`);
  console.log(`  Verify: sqlite3 data/sequelize.sqlite "SELECT role, type, content FROM ChatMessages WHERE chatId = '${chatId}' ORDER BY timestamp;"`);

  try {
    await mcpClient.disconnect();
    console.log('  MCP disconnected');
  } catch {
    // ignore
  }

  // Summary
  console.log('\n' + '='.repeat(60));
  console.log('Test Summary');
  console.log('='.repeat(60));
  console.log(`  Passed: ${passed}`);
  console.log(`  Failed: ${failed}`);
  console.log(`  Total:  ${passed + failed}`);
  console.log('='.repeat(60));

  process.exit(failed > 0 ? 1 : 0);
}

// Run
runTest().catch((error) => {
  console.error('Test suite failed:', error);
  process.exit(1);
});
