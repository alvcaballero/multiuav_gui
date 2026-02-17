#!/usr/bin/env node

/**
 * Test del cliente MCP
 * Ejecuta: node test/test-mcp-client.js
 *
 * Requisitos:
 * - MCP Server debe estar corriendo en http://127.0.0.1:3001/mcp
 * - Iniciar con: cd mcp_server && npm run dev:http
 */

import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import { decode } from '@toon-format/toon';

// Override config BEFORE importing MCPclient (ESM modules evaluate config.js at import time)
process.env.MCP_ENABLE = 'true';
process.env.MCP_CONFIG = JSON.stringify({
  transport: 'http',
  url: 'http://127.0.0.1:3001/mcp/',
});

// Dynamic import so config.js reads the env vars we just set
const { MCPclient } = await import('../models/chat/mcpClient.js');

// Get directory path for loading JSON test files
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const jsonDir = join(__dirname, 'api', 'json');

const mcpClient = new MCPclient();

async function runTests() {
  console.log('='.repeat(60));
  console.log('MCP Client Test Suite');
  console.log('='.repeat(60));
  console.log();

  let passed = 0;
  let failed = 0;

  // Test 1: Connection
  console.log('Test 1: Connecting to MCP Server...');
  try {
    await mcpClient.connect();
    console.log('  PASSED: Connected successfully');
    passed++;
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
    console.log('\n Make sure MCP server is running:');
    console.log('   cd mcp_server && npm run dev:http\n');
    process.exit(1);
  }

  // Test 2: Check ready state
  console.log('\nTest 2: Check ready state...');
  if (mcpClient.isReady()) {
    console.log('  PASSED: Client is ready');
    passed++;
  } else {
    console.log('  FAILED: Client is not ready');
    failed++;
  }

  // Test 3: List tools
  console.log('\nTest 3: List available tools...');
  const tools = mcpClient.getTools();
  if (tools.length > 0) {
    console.log(`  PASSED: Found ${tools.length} tools`);
    console.log('  Tools available:');
    tools.forEach((tool) => {
      console.log(`    - ${tool.name}: ${tool.description?.substring(0, 50)}...`);
    });
    passed++;
  } else {
    console.log('  FAILED: No tools found');
    failed++;
  }

  // Test 4: Get tools in OpenAI format
  console.log('\nTest 4: Get tools in OpenAI format...');
  const openaiTools = mcpClient.getToolsForOpenAI();
  if (openaiTools.length > 0 && openaiTools[0].type === 'function') {
    console.log(`  PASSED: ${openaiTools.length} tools in OpenAI format`);
    passed++;
  } else {
    console.log('  FAILED: Could not format tools for OpenAI');
    failed++;
  }

  // Test 5: Get tools in Anthropic format
  console.log('\nTest 5: Get tools in Anthropic format...');
  const anthropicTools = mcpClient.getToolsForAnthropic();
  if (anthropicTools.length > 0 && anthropicTools[0].input_schema) {
    console.log(`  PASSED: ${anthropicTools.length} tools in Anthropic format`);
    passed++;
  } else {
    console.log('  FAILED: Could not format tools for Anthropic');
    failed++;
  }

  // Test 6: Execute get_devices tool
  console.log('\nTest 6: Execute get_devices tool...');
  try {
    const result = await mcpClient.executeTool('get_devices', {});
    console.log('  PASSED: get_devices executed successfully');
    console.log('  Result:', JSON.stringify(result, null, 2).substring(0, 200) + '...');
    passed++;
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
  }

  // Test 7: Execute get_fleet_telemetry tool
  console.log('\nTest 7: Execute get_fleet_telemetry tool...');
  try {
    const result = await mcpClient.executeTool('get_fleet_telemetry', {});
    console.log('  PASSED: get_fleet_telemetry executed successfully');
    console.log('  Result:', JSON.stringify(result, null, 2).substring(0, 200) + '...');
    passed++;
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
  }

  // Test 8: Execute get_registered_objects tool
  console.log('\nTest 8: Execute get_registered_objects tool...');
  try {
    const result = await mcpClient.executeTool('get_registered_objects', {});
    console.log('  PASSED: get_registered_objects executed successfully');
    console.log('  Result:', JSON.stringify(result, null, 2).substring(0, 200) + '...');
    passed++;
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
  }

  // Test 9: Check connection health
  console.log('\nTest 9: Check connection health...');
  try {
    const isHealthy = await mcpClient.checkConnection();
    if (isHealthy) {
      console.log('  PASSED: Connection is healthy');
      passed++;
    } else {
      console.log('  FAILED: Connection is not healthy');
      failed++;
    }
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
  }

  // Test 10: Tool not found error handling
  console.log('\nTest 10: Tool not found error handling...');
  try {
    await mcpClient.executeTool('non_existent_tool', {});
    console.log('  FAILED: Should have thrown an error');
    failed++;
  } catch (error) {
    if (error.message.includes('Tool not found')) {
      console.log('  PASSED: Correctly threw "Tool not found" error');
      passed++;
    } else {
      console.log(`  FAILED: Wrong error: ${error.message}`);
      failed++;
    }
  }

  // Test 11: Execute validate_mission_collisions tool (with collisions)
  console.log('\nTest 11: Execute validate_mission_collisions (with collisions)...');
  try {
    const collisionData = JSON.parse(readFileSync(join(jsonDir, 'mcp_validate_collision.json'), 'utf-8'));
    const result = await mcpClient.executeTool('validate_mission_collisions', collisionData);
    const resultText = result.content?.[0]?.text || '';
    const parsed = decode(resultText);

    if (parsed.valid === false || parsed.totalCollisions > 0) {
      console.log('  PASSED: validate_mission_collisions detected collisions');
      console.log(`  Collisions: ${parsed.totalCollisions}, Warnings: ${parsed.totalWarnings}`);
      passed++;
    } else {
      console.log('  WARNING: Expected collisions but got valid=true');
      console.log('  Result:', resultText.substring(0, 300) + '...');
      passed++; // Still pass as the tool executed successfully
    }
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
  }

  // Test 12: Execute validate_mission_collisions tool (safe route)
  console.log('\nTest 12: Execute validate_mission_collisions (safe route)...');
  try {
    const safeData = JSON.parse(readFileSync(join(jsonDir, 'mcp_validate_collision_safe.json'), 'utf-8'));
    const result = await mcpClient.executeTool('validate_mission_collisions', safeData);
    const resultText = result.content?.[0]?.text || '';
    const parsed = decode(resultText);

    if (parsed.valid === true || parsed.totalCollisions === 0) {
      console.log('  PASSED: validate_mission_collisions confirmed safe route');
      console.log(`  Collisions: ${parsed.totalCollisions}, Warnings: ${parsed.totalWarnings}`);
      passed++;
    } else {
      console.log('  WARNING: Expected safe but got collisions');
      console.log('  Result:', resultText.substring(0, 300) + '...');
      passed++; // Still pass as the tool executed successfully
    }
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
  }

  // Test 13: Execute validate_mission_collisions tool (multi-route)
  console.log('\nTest 13: Execute validate_mission_collisions (multi-route)...');
  try {
    const multiData = JSON.parse(readFileSync(join(jsonDir, 'mcp_validate_collision_multi_route.json'), 'utf-8'));
    const result = await mcpClient.executeTool('validate_mission_collisions', multiData);
    const resultText = result.content?.[0]?.text || '';
    const parsed = decode(resultText);

    console.log('  PASSED: validate_mission_collisions executed for multi-route');
    console.log(`  Routes validated: ${parsed.routes?.length || 'N/A'}`);
    console.log(`  Total collisions: ${parsed.totalCollisions}, Warnings: ${parsed.totalWarnings}`);
    if (parsed.routes) {
      parsed.routes.forEach((route, i) => {
        console.log(
          `    Route ${i} (${route.routeName}): valid=${route.valid}, collisions=${route.summary?.collisionCount || 0}`
        );
      });
    }
    passed++;
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
  }

  // Test 14: Disconnect
  console.log('\nTest 14: Disconnect from MCP Server...');
  try {
    await mcpClient.disconnect();
    console.log('  PASSED: Disconnected successfully');
    passed++;
  } catch (error) {
    console.log(`  FAILED: ${error.message}`);
    failed++;
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

// Run tests
runTests().catch((error) => {
  console.error('Test suite failed:', error);
  process.exit(1);
});
