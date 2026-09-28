// Minimal, whitelisted arithmetic expression evaluator for the insem/wtsem
// `.yaml` mini-DSL (e.g. `"rotor_diameter/2 - hub_radius"`,
// `"hub_radius*sin(radians(120))"`). Deliberately NOT a general-purpose
// expression library (mathjs, expr-eval, ...) — `expr-eval` was tried first
// and dropped: it has unpatched prototype-pollution/code-execution CVEs
// (GHSA-8gw3-rxh4-v6jx, GHSA-jc85-fpwf-qm7x, GHSA-q9v2-7m5w-4693, no fix
// available). This mirrors the reference implementation's own approach
// (`insem.py`'s `ev()`: a whitelisted AST walk, never Python `eval`) — a
// hand-rolled recursive-descent parser over a fixed grammar can't reach
// `constructor`/`__proto__`/arbitrary JS the way a general evaluator can,
// because property/index access was never implemented, not just blocked.

const FUNCTIONS = {
  sin: Math.sin,
  cos: Math.cos,
  tan: Math.tan,
  asin: Math.asin,
  acos: Math.acos,
  atan: Math.atan,
  atan2: Math.atan2,
  sqrt: Math.sqrt,
  hypot: Math.hypot,
  floor: Math.floor,
  ceil: Math.ceil,
  min: Math.min,
  max: Math.max,
  abs: Math.abs,
  radians: (deg) => (deg * Math.PI) / 180,
  degrees: (rad) => (rad * 180) / Math.PI,
};

const CONSTANTS = { pi: Math.PI };

const TOKEN_RE = /\s*(?:(\d+(?:\.\d+)?)|([a-zA-Z_][a-zA-Z0-9_]*)|([+\-*/^(),]))/y;

function tokenize(source) {
  const tokens = [];
  TOKEN_RE.lastIndex = 0;
  let index = 0;
  while (index < source.length) {
    TOKEN_RE.lastIndex = index;
    const match = TOKEN_RE.exec(source);
    if (!match || match[0].length === 0) {
      throw new Error(`Unexpected character at position ${index}: ${JSON.stringify(source[index])}`);
    }
    const [full, number, ident, op] = match;
    if (number !== undefined) tokens.push({ type: 'number', value: Number(number) });
    else if (ident !== undefined) tokens.push({ type: 'ident', value: ident });
    else if (op !== undefined) tokens.push({ type: 'op', value: op });
    index += full.length;
  }
  tokens.push({ type: 'eof' });
  return tokens;
}

// Grammar (precedence low → high): expr := term (('+'|'-') term)*
// term := power (('*'|'/') power)* ; power := unary ('^' power)? (right-assoc)
// unary := ('-'|'+')? primary
// primary := number | ident | ident '(' expr (',' expr)* ')' | '(' expr ')'
class Parser {
  constructor(tokens, scope) {
    this.tokens = tokens;
    this.pos = 0;
    this.scope = scope;
  }

  peek() {
    return this.tokens[this.pos];
  }

  next() {
    return this.tokens[this.pos++];
  }

  expectOp(op) {
    const token = this.next();
    if (token.type !== 'op' || token.value !== op) {
      throw new Error(`Expected "${op}" but got ${JSON.stringify(token.value ?? token.type)}`);
    }
  }

  parseExpression() {
    let value = this.parseTerm();
    for (;;) {
      const token = this.peek();
      if (token.type === 'op' && (token.value === '+' || token.value === '-')) {
        this.next();
        const rhs = this.parseTerm();
        value = token.value === '+' ? value + rhs : value - rhs;
      } else {
        return value;
      }
    }
  }

  parseTerm() {
    let value = this.parsePower();
    for (;;) {
      const token = this.peek();
      if (token.type === 'op' && (token.value === '*' || token.value === '/')) {
        this.next();
        const rhs = this.parsePower();
        value = token.value === '*' ? value * rhs : value / rhs;
      } else {
        return value;
      }
    }
  }

  parsePower() {
    const base = this.parseUnary();
    const token = this.peek();
    if (token.type === 'op' && token.value === '^') {
      this.next();
      const exponent = this.parsePower(); // right-associative
      return base ** exponent;
    }
    return base;
  }

  parseUnary() {
    const token = this.peek();
    if (token.type === 'op' && (token.value === '-' || token.value === '+')) {
      this.next();
      const value = this.parseUnary();
      return token.value === '-' ? -value : value;
    }
    return this.parsePrimary();
  }

  parsePrimary() {
    const token = this.next();
    if (token.type === 'number') return token.value;
    if (token.type === 'op' && token.value === '(') {
      const value = this.parseExpression();
      this.expectOp(')');
      return value;
    }
    if (token.type === 'ident') {
      const name = token.value;
      if (this.peek().type === 'op' && this.peek().value === '(') {
        this.next();
        const args = [];
        if (!(this.peek().type === 'op' && this.peek().value === ')')) {
          args.push(this.parseExpression());
          while (this.peek().type === 'op' && this.peek().value === ',') {
            this.next();
            args.push(this.parseExpression());
          }
        }
        this.expectOp(')');
        if (!Object.hasOwn(FUNCTIONS, name)) {
          throw new Error(`Unknown function "${name}"`);
        }
        return FUNCTIONS[name](...args);
      }
      if (Object.hasOwn(CONSTANTS, name)) return CONSTANTS[name];
      // Object.hasOwn (not `in`/bare property access) so a variable named
      // "constructor"/"__proto__" in the YAML can't read back inherited
      // Object.prototype members — the whole class of bug expr-eval shipped.
      if (this.scope && Object.hasOwn(this.scope, name)) return this.scope[name];
      throw new Error(`Undefined variable "${name}"`);
    }
    throw new Error(`Unexpected token ${JSON.stringify(token.value ?? token.type)}`);
  }
}

/**
 * Evaluates a whitelisted arithmetic expression string against a flat
 * numeric scope (e.g. `"rotor_diameter/2 - hub_radius"`). Throws with a
 * clear message on any undefined variable, unknown function, or syntax
 * error — never silently returns NaN/undefined.
 */
export function evaluateExpression(source, scope) {
  const tokens = tokenize(source);
  const parser = new Parser(tokens, scope);
  const value = parser.parseExpression();
  if (parser.peek().type !== 'eof') {
    throw new Error(`Unexpected trailing input near ${JSON.stringify(parser.peek().value)}`);
  }
  if (typeof value !== 'number' || Number.isNaN(value)) {
    throw new Error(`Expression did not evaluate to a number: ${source}`);
  }
  return value;
}
