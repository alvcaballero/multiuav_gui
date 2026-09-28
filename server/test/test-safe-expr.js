import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { evaluateExpression } from '../models/markers/safeExpr.js';

describe('evaluateExpression', () => {
  test('basic arithmetic with variables', () => {
    assert.equal(evaluateExpression('rotor_diameter/2 - hub_radius', { rotor_diameter: 120, hub_radius: 1.8 }), 58.2);
  });

  test('nested function calls (radians -> sin)', () => {
    const got = evaluateExpression('hub_radius*sin(radians(120))', { hub_radius: 1.8 });
    assert.ok(Math.abs(got - 1.8 * Math.sin((120 * Math.PI) / 180)) < 1e-9);
  });

  test('unary minus on a variable', () => {
    assert.equal(evaluateExpression('-shaft_tilt_deg', { shaft_tilt_deg: 5 }), -5);
  });

  test('operator precedence and parentheses', () => {
    assert.equal(evaluateExpression('2 + 3 * 4', {}), 14);
    assert.equal(evaluateExpression('(2 + 3) * 4', {}), 20);
  });

  test('the pi constant', () => {
    assert.ok(Math.abs(evaluateExpression('2*pi*radius', { radius: 1 }) - 2 * Math.PI) < 1e-9);
  });

  test('rejects an undefined variable', () => {
    assert.throws(() => evaluateExpression('foo + 1', {}), /Undefined variable "foo"/);
  });

  test('rejects an unknown function', () => {
    assert.throws(() => evaluateExpression('wobble(1)', {}), /Unknown function "wobble"/);
  });

  test('rejects malformed syntax', () => {
    assert.throws(() => evaluateExpression('1 + ', {}));
    assert.throws(() => evaluateExpression('(1 + 2', {}));
  });

  test('security: cannot read Object.prototype members via scope', () => {
    assert.throws(() => evaluateExpression('constructor', {}), /Undefined variable/);
    assert.throws(() => evaluateExpression('__proto__', {}), /Undefined variable/);
    assert.throws(() => evaluateExpression('toString', {}), /Undefined variable/);
  });

  test('security: no property access or indexing syntax exists', () => {
    assert.throws(() => evaluateExpression('x.y', { x: 1 }));
    assert.throws(() => evaluateExpression('x[0]', { x: 1 }));
  });
});
