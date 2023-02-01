const execShPromise = require("exec-sh").promise;

// run interactive bash shell
const ros_bridge_run = async () => {
  let out;
  

  try {
    out = await execShPromise('roslaunch rosbridge_server rosbridge_websocket.launch &', true);
    console.log("Hello World" + out)
  } catch (e) {
    console.log('Error: ', e);
    console.log('Stderr: ', e.stderr);
    console.log('Stdout: ', e.stdout);

    return e;
  }

  console.log('out: ', out.stdout, out.stderr);
}

// run interactive bash shell
const ros_bridge_close = async () => {
    let out;
  
    try {
      out = await execShPromise('kill -9 '+pid_value, true);
    } catch (e) {
      console.log('Error: ', e);
      console.log('Stderr: ', e.stderr);
      console.log('Stdout: ', e.stdout);
  
      return e;
    }
  
    console.log('out: ', out.stdout, out.stderr);
  }