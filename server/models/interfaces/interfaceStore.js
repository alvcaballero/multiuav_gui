// Files of the interfaces (the "categories" of a device): config/devices/interfaces/<key>.yaml,
// one per interface. The FILE NAME is the key, so the file system keeps keys unique, and an edit
// rewrites only its own file. IO only: what a file must contain is interfaceCatalog.js.
import { readdirSync, readFileSync } from 'node:fs';
import { writeFile, unlink } from 'node:fs/promises';
import path from 'node:path';
import { parseDocument, stringify } from 'yaml';

const EXT = '.yaml';
export const INTERFACE_KEY = /^[A-Za-z0-9_]+$/;

export const isInterfaceKey = (key) => typeof key === 'string' && INTERFACE_KEY.test(key);

/** The file of `key`, or an Error when `key` could escape `dir` (`../`) or is not a key. */
function fileOf(dir, key) {
  if (!isInterfaceKey(key)) throw new Error(`'${key}' is not a valid interface key (${INTERFACE_KEY})`);
  return path.join(dir, `${key}${EXT}`);
}

/**
 * Every `*.yaml` of `dir`, parsed. Other extensions (`.bak`, `.orig`) are ignored; a name that is
 * not a key, or a file YAML cannot parse, is reported (with its line) and left out.
 * @returns {{ raw: Record<string, object>, problems: {path: string[], message: string}[] }}
 */
export function readInterfaceDir(dir) {
  const raw = {};
  const problems = [];
  let files;
  try {
    files = readdirSync(dir).filter((file) => file.endsWith(EXT)).sort();
  } catch (err) {
    return { raw, problems: [{ path: [], message: `cannot read ${dir}: ${err.message}` }] };
  }
  for (const file of files) {
    const key = file.slice(0, -EXT.length);
    if (!isInterfaceKey(key)) {
      problems.push({ path: [key], message: `file '${file}' is not named after a valid interface key (${INTERFACE_KEY})` });
      continue;
    }
    const doc = parseDocument(readFileSync(path.join(dir, file), 'utf8'));
    if (doc.errors.length) {
      for (const err of doc.errors) problems.push({ path: [key], message: `${file}: ${err.message}` });
      continue;
    }
    raw[key] = doc.toJS() ?? {};
  }
  return { raw, problems };
}

/** Writes `def` as the file of `key` (its comments are lost: the file is generated). */
export async function writeInterfaceFile(dir, key, def) {
  await writeFile(fileOf(dir, key), stringify(def));
}

export async function deleteInterfaceFile(dir, key) {
  await unlink(fileOf(dir, key));
}
