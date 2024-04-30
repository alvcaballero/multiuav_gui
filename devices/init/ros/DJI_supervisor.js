import { exec, spawn } from 'child_proces';

proc = spawn('M300-sim');

proc.stdout.on('data', function (res) {
  console.log('Data received: ' + res);
});
