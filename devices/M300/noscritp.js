import child_process, { spawn } from 'child_process';
import util from 'util';
import { parse } from 'yaml';
import { readFileSync } from 'fs';
import { resolve } from 'path';
const __dirname = new URL('.', import.meta.url).pathname;

const exec = util.promisify(child_process.exec);

let mainProcessRunning = false;
let RSTPisRunning = false;

console.log('----------Starting script...');

const M300_real = 'ls ~';
const M300_sim = 'ls ~/work/';

let configScript = M300_real ? M300_real : M300_sim;

// -----------------  utils funtions -----------------
function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

await sleep(5000);

function readYAML(path) {
  let fileContents = null;
  try {
    fileContents = readFileSync(resolve(__dirname, path), 'utf8');
  } catch (e) {
    console.log(e);
  }
  return parse(fileContents);
}

const config = readYAML('./config.yaml');
/* uav_id: uav_14
 * real: M300_real
 * M300_real:
 * M300_sim:
 */

try {
  console.log('---------- set out  device  M300-real...');

  const { stdout, stderr } = await exec(configScript);
  console.log('stdout:', stdout);
  console.log('stderr:', stderr);
  if (stdout.includes('ERROR')) {
    console.error('Error detected in logs. Restarting script...');
  }
} catch (e) {
  console.error(e); // should contain code (exit code) and signal (that caused the termination).
}

await sleep(5000);
async function mainProcess() {
  try {
    console.log('---------- set out  device start - main program...');
    const { stdout, stderr } = await exec('bash -c "source ~/.nvm/nvm.sh && nvm use 21 && pm2 start notest"');
    console.log('stdout:', stdout);
    console.log('stderr:', stderr);
  } catch (e) {
    console.error(e); // should contain code (exit code) and signal (that caused the termination).
  }
  await sleep(5000);
}
// Start the monitoring loop
//setInterval(monitorLogs, 5000); // Check logs every 5 seconds

console.log('---------- span process...');
const ls = spawn('bash', ['-c', 'source ~/.nvm/nvm.sh && nvm use 21 && pm2 logs notest']);

ls.stdout.on('data', (data) => {
  console.log(`spawn stdout: ${data}`);
});

ls.stderr.on('data', (data) => {
  console.error(`spawn stderr: ${data}`);
  if (data.includes('end mission')) {
    console.error('Error detected in logs. Restarting script...');
  }
  // error waypoints so close or other to upload new mission
  if (data.includes('Error')) {
    console.error('Error detected in logs. Restarting script...');
  }
  // reset node
});

ls.on('close', (code) => {
  console.log(`spawn child process exited with code ${code}`);
});

await sleep(5000);

console.log('----------  set up camera...');

try {
  const { stdout, stderr } = await exec('echo one; sleep 1; echo two; sleep 1');
  console.log('stdout:', stdout);
  console.log('stderr:', stderr);
} catch (e) {
  console.error(e); // should contain code (exit code) and signal (that caused the termination).
}

await sleep(5000);

console.log('----------  set  recise script...');
try {
  const { stdout, stderr } = await exec('echo one; sleep 1; echo two; sleep 1');
  console.log('stdout:', stdout);
  console.log('stderr:', stderr);
} catch (e) {
  console.error(e); // should contain code (exit code) and signal (that caused the termination).
}
await sleep(5000);

console.log('----------  set  rstp rost launch...');

try {
  const { stdout, stderr } = await exec('echo one; sleep 1; echo two; sleep 1');
  console.log('stdout:', stdout);
  console.log('stderr:', stderr);
} catch (e) {
  console.error(e); // should contain code (exit code) and signal (that caused the termination).
}
await sleep(5000);

async function SetupCamera() {
  try {
    const { stdout, stderr } = await exec('echo one; sleep 1; echo two; sleep 1');
    console.log('stdout:', stdout);
    console.log('stderr:', stderr);
  } catch (e) {
    console.error(e); // should contain code (exit code) and signal (that caused the termination).
  }
  await sleep(5000);
  try {
    const { stdout, stderr } = await exec('echo one; sleep 1; echo two; sleep 1');
    console.log('stdout:', stdout);
    console.log('stderr:', stderr);
  } catch (e) {
    console.error(e); // should contain code (exit code) and signal (that caused the termination).
  }
  await sleep(5000);
}

async function monitorCheckRTSP() {
  try {
    const { stdout, stderr } = await exec('echo one; sleep 1; echo two; sleep 1');
    console.log('stdout:', stdout);
    console.log('stderr:', stderr);
    if (stdout.includes('ERROR')) {
      console.error('Error detected in Monitor check rtsp. Restarting script...');
      if (mainProcessRunning && RSTPisRunning) {
        SetupCamera();
      }
    }
  } catch (e) {
    console.error(e); // should contain code (exit code) and signal (that caused the termination).
  }
}

setInterval(monitorCheckRTSP, 5000); // Check logs every 5 seconds

//ls.stdin.pause();
//ls.kill();
