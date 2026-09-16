#!/usr/bin/env node

/**
 * Test del sistema de logging
 * Ejecuta: node test-logger.js
 */

import { logger, logHelpers, wsLogger, deviceLogger, rosLogger, createCustomLogger, chalk } from '../common/logger.js';

console.log('🚀 Iniciando pruebas del sistema de logging...\n');

// Test 1: Logger básico
console.log('📋 Test 1: Logger básico');
logger.info('Mensaje de información');
logger.warn('Mensaje de advertencia');
logger.error('Mensaje de error');
logger.debug('Mensaje de debug (solo visible si LOG_LEVEL=debug)');

// Test 2: Logger con metadatos
console.log('\n📋 Test 2: Logger con metadatos');
logger.info('Operación completada', {
  operacion: 'test',
  usuario: 'admin',
  duracion: '125ms',
  items: 42,
});

// Test 3: Logger de sistema
console.log('\n📋 Test 3: Logger de sistema');
logHelpers.system.info('Sistema inicializado');
logHelpers.system.warn('Memoria alta detectada', { usage: '85%' });
logHelpers.system.error('Error de conexión', new Error('Connection refused'));

// Test 4: Logger de API
console.log('\n📋 Test 4: Logger de API');
logHelpers.api.request('GET', '/api/devices', '192.168.1.100');
logHelpers.api.response('GET', '/api/devices', 200, 45);
logHelpers.api.error('POST', '/api/commands', new Error('Validation failed'));

// Test 5: Logger de WebSocket
console.log('\n📋 Test 5: Logger de WebSocket');
logHelpers.ws.connect('client-123');
logHelpers.ws.message('client-123', 'Hello server');
logHelpers.ws.broadcast('System notification', 5);
logHelpers.ws.disconnect('client-123');

// Test 6: Logger de dispositivos
console.log('\n📋 Test 6: Logger de dispositivos');
logHelpers.device.connect('uav-001', 'M300-Drone');
logHelpers.device.position('uav-001', 'M300-Drone', { lat: 40.7128, lng: -74.006, alt: 100 });
logHelpers.device.command('uav-001', 'M300-Drone', 'takeoff');
logHelpers.device.status('uav-001', 'M300-Drone', 'flying');

// Test 7: Logger de ROS
console.log('\n📋 Test 7: Logger de ROS');
logHelpers.ros.connect('mavros_node');
logHelpers.ros.publish('/mavros/setpoint_position/local', 'mavros_node');
logHelpers.ros.message('/mavros/global_position/global', 'mavros_node', 'NavSatFix');

// Test 8: Logger personalizado
console.log('\n📋 Test 8: Logger personalizado');
const testLogger = createCustomLogger('TEST', {
  level: 'info',
  color: chalk.magenta,
  filename: 'test.log',
});

testLogger.info('Logger personalizado funcionando');
testLogger.warn('Advertencia del logger personalizado');

// Test 9: Logging de errores con stack trace
console.log('\n📋 Test 9: Logging de errores con stack trace');
try {
  throw new Error('Error de prueba con stack trace');
} catch (error) {
  logger.error('Error capturado en test', error);
}

// Test 10: Logging de performance
console.log('\n📋 Test 10: Logging de performance');
const start = Date.now();
setTimeout(() => {
  const duration = Date.now() - start;
  logger.info('Operación completada', {
    operation: 'timeout-test',
    duration: `${duration}ms`,
    success: true,
  });
}, 100);

// Test 11: Loggers específicos directos
console.log('\n📋 Test 11: Loggers específicos directos');
wsLogger.info('Mensaje directo del WebSocket logger');
deviceLogger.info('Mensaje directo del device logger', { deviceId: 'test-001' });
rosLogger.info('Mensaje directo del ROS logger', { topic: '/test/topic' });

console.log('\n✅ Pruebas del logger completadas!');
console.log('📁 Revisa los archivos en la carpeta ./logs/');
console.log('📁 Archivos generados:');
console.log('   - combined.log (todos los logs)');
console.log('   - error.log (solo errores)');
console.log('   - websocket.log (logs de WebSocket)');
console.log('   - devices.log (logs de dispositivos)');
console.log('   - ros.log (logs de ROS)');
console.log('   - test.log (logger personalizado)');

// Mostrar información de configuración
console.log('\n⚙️  Configuración actual:');
console.log(`   - LOG_LEVEL: ${process.env.LOG_LEVEL || 'info'}`);
console.log(`   - WS_LOG_LEVEL: ${process.env.WS_LOG_LEVEL || 'info'}`);
console.log(`   - DEVICE_LOG_LEVEL: ${process.env.DEVICE_LOG_LEVEL || 'info'}`);
console.log(`   - ROS_LOG_LEVEL: ${process.env.ROS_LOG_LEVEL || 'info'}`);

setTimeout(() => {
  process.exit(0);
}, 200);
