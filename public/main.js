// var socket = io.connect("http://193.147.161.33:8080", { forceNew: true });

// var socket = io.connect("http://10.20.20.100:3000", { forceNew: true });
var socket = io.connect("http://localhost:3000", { forceNew: true });

socket.on("messages", function (data) {
  console.log(data.id);
  if (data.id == 2) {
    console.log("Mision recibida");
      rendermission(data);
  } else if (data.id == 4) {
    console.log(data.uav_id+" Launch mission received");
      renderCommandMission(data);      
  } else {
      render(data);
  }
  console.log(data);
  
});

function render(data) {
  var array = [
    data,
  ];
  var html = array
    .map(function (elem, index) {
      return `<div>
              <span><b>UAV ID:</b></span>
              <em>${elem.uav_id}</em>
              </div>
              <div>
              <span><b>Pose:</b></span>
              <em>${elem.latitude}</em>, 
              <em>${elem.longitude}</em>
              </div>
              <div>
              <span><b>Altitude:</b></span>
              <em>${elem.altitude}</em>
              </div>
              <div>
              <span><b>Altitude_MSL:</b></span>
              <em>${elem.altitude_MSL}</em>
              </div>
              <div>
              <span><b>Altitude_relative:</b></span>
              <em>${elem.altitude_rel}</em>
              </div>                            
              <div>
              <span><b>Vel_x:</b></span>
              <em>${elem.vel_x}</em>
              </div>
              <div>
              <span><b>Vel_y:</b></span>
              <em>${elem.vel_y}</em>
              </div>
              <div>
              <span><b>Velocity:</b></span>
              <em>${elem.groundspeed}</em>
              </div>
              <div>
              <span><b>Orientation:</b></span>
              <em>${elem.hdg}</em>
              </div>              
              <div>
              <span><b>Battery:</b></span>
              <em>${elem.battery}</em>
            </div>
            <div>
              <span><b>Info:</b></span>
              <em>${elem.info}</em>
            </div>
            `;
    })
    .join(" ");

  document.getElementById("messages").innerHTML = html;
console.log(data.uav_id);
// let n = data.uav_id;
// nstring=n.toString();
// console.log(n);
// lo que viene a continuacion es para mostrar en la tabla del cliente la info que ya viene mapeada en el HTML. No es necesario.
  // let uav_ns = data.uav_id;
  // var showData = document.getElementById(uav_ns).cells;
  // showData[0].innerHTML = data.uav_id;
  // showData[1].innerHTML = data.altitude_rel;
  // showData[2].innerHTML = data.groundspeed;
  // showData[3].innerHTML = data.battery;
}

function rendermission(data) {
  var array = [
    data,
  ];
  
  if(array[0].uav_n==1){
    // console.log("UN SOLO UAV");
    var html = array
    .map(function (elem, index) {
      return `<div>
              <span><b>Nº de UAVs:</b></span>
              <em>${elem.uav_n}</em>
              </div>
              <div>
              <span><b>Frame_id:</b></span>
              <em>${elem.frame_id}</em>              
              </div>              
              <span><b>Waypoints:</b></span>
              <em>${elem.uav_1.wp_n}</em>
              </div>
              <div>
              <span><b>WP_0:</b></span>
              <em>${elem.uav_1.wp_0}</em>
              </div>
              <div>
              <span><b>WP_1:</b></span>
              <em>${elem.uav_1.wp_1}</em>
              </div>
              <div>
              <span><b>WP_2:</b></span>
              <em>${elem.uav_1.wp_2}</em>
              </div>
              <div>
              <span><b>WP_3:</b></span>
              <em>${elem.uav_1.wp_3}</em>
              </div>
              <div>
              <span><b>WP_4:</b></span>
              <em>${elem.uav_1.wp_4}</em>
              </div>
              <div>
              <span><b>WP_5:</b></span>
              <em>${elem.uav_1.wp_5}</em>
              </div>
              <div>
              <span><b>WP_6:</b></span>
              <em>${elem.uav_1.wp_6}</em>
              </div>
              <div>
              <span><b>WP_7:</b></span>
              <em>${elem.uav_1.wp_7}</em>
              </div>
              <div>
              <span><b>WP_8:</b></span>
              <em>${elem.uav_1.wp_8}</em>
              </div>
            `
            ;
    })
    .join(" ");

  document.getElementById("messages").innerHTML = html;

  }
  // var html = array
  //   .map(function (elem, index) {
  //     return `<div>
  //             <span><b>Nº de UAVs:</b></span>
  //             <em>${elem.uav_n}</em>
  //             </div>
  //             <div>
  //             <span><b>Frame_id:</b></span>
  //             <em>${elem.frame_id}</em>              
  //             </div>              
  //             <span><b>Waypoints:</b></span>
  //             <em>${elem.uav_1.wp_n}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_0:</b></span>
  //             <em>${elem.uav_1.wp_0}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_1:</b></span>
  //             <em>${elem.uav_1.wp_1}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_2:</b></span>
  //             <em>${elem.uav_1.wp_2}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_3:</b></span>
  //             <em>${elem.uav_1.wp_3}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_4:</b></span>
  //             <em>${elem.uav_1.wp_4}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_5:</b></span>
  //             <em>${elem.uav_1.wp_5}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_6:</b></span>
  //             <em>${elem.uav_1.wp_6}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_7:</b></span>
  //             <em>${elem.uav_1.wp_7}</em>
  //             </div>
  //             <div>
  //             <span><b>WP_8:</b></span>
  //             <em>${elem.uav_1.wp_8}</em>
  //             </div>
  //           `
  //           ;
  //   })
  //   .join(" ");

  // document.getElementById("messages").innerHTML = html;

  if(array[0].uav_n==2){
    // console.log("DOS UAVS");
    var html = array
    .map(function (elem, index) {
      return `<div>
              <span><b>Nº de UAVs:</b></span>
              <em>${elem.uav_n}</em>
              </div>
              <div>
              <span><b>Frame_id:</b></span>
              <em>${elem.frame_id}</em>              
              </div>
              <div>
              <span><b>UAV 1:</b></span>
              </div>
              <div>               
              <span><b>Waypoints:</b></span>
              <em>${elem.uav_1.wp_n}</em>
              </div>
              <div>
              <span><b>WP_0:</b></span>
              <em>${elem.uav_1.wp_0}</em>
              </div>
              <div>
              <span><b>WP_1:</b></span>
              <em>${elem.uav_1.wp_1}</em>
              </div>
              <div>
              <span><b>WP_2:</b></span>
              <em>${elem.uav_1.wp_2}</em>
              </div>
              <div>
              <span><b>UAV 2:</b></span>
              </div>
              <div>               
              <span><b>Waypoints:</b></span>
              <em>${elem.uav_2.wp_n}</em>
              </div> 
              <div>
              <span><b>WP_0:</b></span>
              <em>${elem.uav_2.wp_0}</em>
              </div>
              <div>
              <span><b>WP_1:</b></span>
              <em>${elem.uav_2.wp_1}</em>
              </div>
              <div>
              <span><b>WP_2:</b></span>
              <em>${elem.uav_2.wp_2}</em>
              </div> 
              <div>
              <span><b>WP_3:</b></span>
              <em>${elem.uav_2.wp_3}</em>
              </div>
              <div>
              <span><b>WP_4:</b></span>
              <em>${elem.uav_2.wp_4}</em>
              </div>
              <div>
              <span><b>WP_5:</b></span>
              <em>${elem.uav_2.wp_5}</em>
              </div>
              <div>
              <span><b>WP_6:</b></span>
              <em>${elem.uav_2.wp_6}</em>
              </div>
              <div>
              <span><b>WP_7:</b></span>
              <em>${elem.uav_2.wp_7}</em>
              </div>
              <div>
              <span><b>WP_8:</b></span>
              <em>${elem.uav_2.wp_8}</em>
              </div>
              <div>
              <span><b>WP_9:</b></span>
              <em>${elem.uav_2.wp_9}</em>
              </div>
            `
            ;
    })
    .join(" ");

  document.getElementById("messages").innerHTML = html;
    
  }
  
}

function addMessage(e) {
  var message = {
    uav_id: document.getElementById("id_uav").value,
    latitude: document.getElementById("lat").value,
    longitude: document.getElementById("lon").value,
    altitude: document.getElementById("alt").value,
    altitude_MSL: document.getElementById("alt_MSL").value,
    altitude_rel: document.getElementById("alt_rel").value,
    vel_x: document.getElementById("vel_x").value,
    vel_y: document.getElementById("vel_y").value,
    groundspeed: document.getElementById("vel").value,
    hdg: document.getElementById("ori").value,
    battery: document.getElementById("bat").value,
    info: document.getElementById("info").value,
    id: 1
  };
    
  socket.emit("new-message", message);
  return false;
}

function requestMission(e) {
  // TBD Esto debe ser dinamico dependiendo del uav que lo solicite
  var message = {
    id: 2,
    uav_id: document.getElementById("id_uav").value
  };

  socket.emit("new-message", message);
  return false;
}

function requestToFly(e) {
  // TBD Esto debe ser dinamico dependiendo del uav que lo solicite
  var message = {
    id: 3,
    uav_id: document.getElementById("id_uav").value
  };

  socket.emit("new-message", message);
  return false;
}

function renderCommandMission(data) {
  var array = [
    data,
  ];
  var html = array
    .map(function (elem, index) {
      return `<div>
              <span><b>UAV ID:</b></span>
              <em>${elem.uav_id}</em>
              </div>
              <div>
              <span><b>Info:</b></span>
              <em>${elem.info}</em>
              </div>
              `;
    })
    .join(" ");

  document.getElementById("messages").innerHTML = html;
  console.log("Launch order received");
}