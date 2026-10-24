// `extends` for a catalog of YAML definitions (interfaces, device models): one definition may name
// another as its parent and only declare what changes. Pure: the caller says how to merge.

/**
 * Resolves every definition of `shaped` (key → definition whose own file is valid) in two passes, so
 * the order of the files does not matter. A definition whose parent does not exist, is invalid, or
 * closes a cycle is left out, and so are its children (their problem says why).
 * @param {Record<string, object>} shaped definitions that passed their own validation
 * @param {Iterable<string>} allKeys every key that has a file, valid or not (tells "invalid" from "unknown")
 * @param {(parent: object, child: object) => object} merge child over its resolved parent
 * @returns {{ resolved: Record<string, object>, problems: {path: string[], message: string}[] }}
 */
export function resolveExtends(shaped, allKeys, merge) {
  const keys = new Set(allKeys);
  const resolved = {};
  const problems = [];
  const failed = new Set([...keys].filter((key) => !shaped[key]));
  const resolve = (key, chain) => {
    if (resolved[key] || failed.has(key)) return resolved[key];
    const def = shaped[key];
    const fail = (message) => {
      problems.push({ path: [key, 'extends'], message });
      failed.add(key);
    };
    if (!def.extends) return (resolved[key] = merge({}, def));
    const parentKey = def.extends;
    if (parentKey === key || chain.includes(parentKey)) return fail(`cycle: ${[...chain, key, parentKey].join(' → ')}`);
    if (!keys.has(parentKey)) return fail(`extends unknown '${parentKey}'`);
    const parent = resolve(parentKey, [...chain, key]);
    if (!parent) return fail(`not loaded because its parent '${parentKey}' is invalid`);
    return (resolved[key] = merge(parent, def));
  };
  for (const key of Object.keys(shaped)) resolve(key, []);
  return { resolved, problems };
}

/** The definition at the top of `key`'s `extends` chain (itself when it extends nothing). */
export function rootOf(raw, key) {
  const seen = new Set();
  let current = key;
  while (raw[current]?.extends && !seen.has(current)) {
    seen.add(current);
    current = raw[current].extends;
  }
  return current;
}
