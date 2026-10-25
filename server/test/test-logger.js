import { test } from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { mkdtempSync, readdirSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

// El logger lee el entorno y abre sus archivos AL IMPORTARSE: cada caso corre en un subproceso propio, con LOG_DIR
// en un directorio temporal, para no escribir nunca en los logs/ reales del server.
const LOGGER = new URL('../common/logger.js', import.meta.url).href;

/** Importa el logger con `env`, corre `body` y espera a que los archivos de `waitFor` tengan contenido. */
const runLogger = (env, body, waitFor = []) => {
  const dir = mkdtempSync(join(tmpdir(), 'muav-logger-'));
  const script = `
    import fs from 'node:fs';
    import * as L from '${LOGGER}';
    ${body}
    const dir = process.env.LOG_DIR;
    const ready = () => ${JSON.stringify(waitFor)}.every((f) => fs.existsSync(dir + '/' + f) && fs.statSync(dir + '/' + f).size > 0);
    for (const t0 = Date.now(); !ready() && Date.now() - t0 < 3000; ) await new Promise((r) => setTimeout(r, 20));
    process.exit(0);
  `;
  const stdout = execFileSync(process.execPath, ['--input-type=module', '-e', script], {
    env: { ...process.env, LOG_FILES: '', LOG_LEVEL: '', FORCE_COLOR: '0', LOG_DIR: dir, ...env },
    encoding: 'utf8',
  });
  const files = Object.fromEntries(readdirSync(dir).map((f) => [f, readFileSync(join(dir, f), 'utf8')]));
  rmSync(dir, { recursive: true, force: true });
  return { stdout, files };
};

test('LOG_FILES=false: solo consola, ningún archivo', () => {
  const { stdout, files } = runLogger({ LOG_FILES: 'false' }, `L.logger.error('boom'); L.missionLogger.info('m');`);
  assert.deepEqual(files, {});
  assert.match(stdout, /boom/);
});

test('por defecto: combined recibe todo, error solo los errores, cada módulo su archivo', () => {
  const { files } = runLogger(
    {},
    `L.logger.info('info-line'); L.logger.error('error-line'); L.missionLogger.info('mission-line');`,
    ['combined.log', 'error.log', 'mission.log']
  );
  assert.match(files['combined.log'], /info-line/);
  assert.match(files['combined.log'], /error-line/);
  assert.match(files['combined.log'], /mission-line/);
  assert.match(files['error.log'], /error-line/);
  assert.doesNotMatch(files['error.log'], /info-line/);
  assert.match(files['mission.log'], /mission-line/);
  assert.doesNotMatch(files['mission.log'], /info-line/);
  assert.deepEqual(JSON.parse(files['mission.log'].trim()).message, 'mission-line');
});

// Sin `tailable` winston numera hacia arriba y, tras un reinicio, la tanda anterior queda fuera de lo que borra
// `maxFiles` (se acumularon combined32..50). La rotación en sí se verificó a mano: acá se fija la configuración.
test('todos los archivos rotan con nombres fijos (tailable) y con límite', () => {
  const { stdout } = runLogger(
    {},
    `const all = [L.logger, L.wsLogger, L.deviceLogger, L.rosLogger, L.chatLogger, L.missionLogger]
       .flatMap((l) => l.transports).filter((t) => t.filename);
     console.log(JSON.stringify(all.map((t) => ({ file: t.filename, tailable: t.tailable, maxFiles: t.maxFiles, maxsize: t.maxsize }))));`
  );
  const transports = JSON.parse(stdout.trim().split('\n').pop());
  assert.ok(transports.length > 0);
  for (const t of transports) {
    assert.equal(t.tailable, true, t.file);
    assert.ok(t.maxFiles > 0 && t.maxsize > 0, t.file);
  }
});

test('createLogger: el label sale en consola y envLevelKey pisa LOG_LEVEL', () => {
  const { stdout } = runLogger(
    { LOG_FILES: 'false', LOG_LEVEL: 'info', MY_LOG_LEVEL: 'debug' },
    `const a = L.createLogger({ label: 'ALPHA', envLevelKey: 'MY_LOG_LEVEL' });
     const b = L.createLogger({ label: 'BETA' });
     a.debug('alpha-debug'); b.debug('beta-debug'); b.info('beta-info');`
  );
  assert.match(stdout, /\[ALPHA\].*alpha-debug/);
  assert.doesNotMatch(stdout, /beta-debug/);
  assert.match(stdout, /\[BETA\].*beta-info/);
});
