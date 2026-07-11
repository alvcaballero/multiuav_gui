/**
 * Test rápido del sistema EventBus
 * Ejecutar con: node test-eventbus.js
 */

import { eventBus, EVENTS } from '../common/eventBus.js';

console.log('\n🧪 Testing EventBus System\n');

// Test 1: Emisión básica
console.log('Test 1: Basic event emission');
eventBus.on(EVENTS.MISSION_CREATED, (data) => {
  console.log('✅ MISSION_CREATED recibido:', data);
});

eventBus.emitSafe(EVENTS.MISSION_CREATED, {
  id: 1,
  name: 'Test Mission',
  status: 'init',
});

// Test 2: Múltiples listeners
console.log('\nTest 2: Multiple listeners');
eventBus.on(EVENTS.EVENT_CREATED, (data) => {
  console.log('✅ Listener 1 recibió EVENT_CREATED:', data.type);
});

eventBus.on(EVENTS.EVENT_CREATED, (data) => {
  console.log('✅ Listener 2 recibió EVENT_CREATED:', data.type);
});

eventBus.emitSafe(EVENTS.EVENT_CREATED, {
  type: 'info',
  deviceId: 1,
  attributes: { message: 'Test event' },
});

// Test 3: Estadísticas
console.log('\nTest 3: EventBus statistics');
const stats = eventBus.getStats();
console.log('📊 Stats:', JSON.stringify(stats, null, 2));

// Test 4: Error handling
console.log('\nTest 4: Error handling in listeners');
eventBus.onSafe(EVENTS.MISSION_UPDATED, (data) => {
  throw new Error('This error should be caught');
});

eventBus.emitSafe(EVENTS.MISSION_UPDATED, { id: 2 });
console.log('✅ Error was handled gracefully');

// Test 5: Cleanup
console.log('\nTest 5: Cleanup');
eventBus.cleanup();
const statsAfterCleanup = eventBus.getStats();
console.log('📊 Stats after cleanup:', JSON.stringify(statsAfterCleanup, null, 2));

console.log('\n✅ All tests passed!\n');
