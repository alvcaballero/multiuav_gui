const { each } = require("async");
const { INSPECT_MAX_BYTES } = require("buffer");
const { on } = require("process");
const { exec } = require("child_process");
const { resolve } = require("dns");
const { rejects } = require("assert");
var dateTime = require('node-datetime');

let MIN_DIST_WP = 500,
circle_set = false;
let kml = "",
kml_info = "",
plan_mission = "",
mission_layers = [],
inspection_plan = [];
let	markerLayer="",
path_limit_layer="",
inspection_init_end = [],
inspection_init_end_markers = [];
let ros = "",
listen_yaw = "";
var uav_pose = [40.615098, -3.731222];
let manualMissionPath = [];
let statusLog = [];

let uav_list = new Array();
var listoftopic
var dt = dateTime.create();
var mode_landing =0;


window.addEventListener('load', () => {
	var wp_blue_icon = L.icon({
		iconUrl: '../assets/css/images/marker-blue.png',
		iconSize: [25, 41],
		iconAnchor: [12, 41],
		shadowUrl: '../assets/css/images/marker-shadow.png',
		iconSize: [25, 41],
		iconAnchor: [12, 41],
		popupAnchor: [0, -41],
	});	
	var point_icon = L.icon({
		iconUrl: '../assets/css/images/point.png',
		iconSize: [32, 32],
		iconAnchor: [16, 16],
	});
	var uav_icon = L.icon({
		iconUrl: '../assets/css/images/uav-marker.png',
		iconSize: [24, 24],
		iconAnchor: [12, 12],
		rotation: 30
	});

	const path2Map = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png'
	const sp_NE = L.latLng(44.175715, -9.688462),
	sp_SW = L.latLng(35.560621, 3.341323),
    SPAIN_BOUNDS = L.latLngBounds(sp_SW, sp_NE);

	let map = L
		.map('mapid')
		.fitBounds(SPAIN_BOUNDS);
	
	let baseMap = L.tileLayer(path2Map, {
		maxZoom: 18,
		minZoom: 7,
		attribution: '<a href="https://www.openstreetmap.org/">OpenStreetMap</a>, ' +
			'<a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>',
	})

	baseMap.addTo(map);

	var editableLayers = new L.FeatureGroup();
    map.addLayer(editableLayers);
    
    var options = {
        position: 'topright',
        draw: {
            polyline: {
                shapeOptions: {
                    // color: rand_color(),
                    weight: 5
                }
            },
			polygon : false,
			// polyline : false,
			rectangle : false,
			circle : false,
			circlemarker: false,
			marker : false
        },
        edit: {
            featureGroup: editableLayers, //REQUIRED!!
            // remove: false
        }
	};
    
    var drawControl = new L.Control.Draw(options);
    map.addControl(drawControl);
    
    map.on(L.Draw.Event.CREATED, function (e) {
        var type = e.layerType,
            layer = e.layer,
			dist = 0;
		console.log(layer._latlngs)

		let wp_list = []

		layer._latlngs.forEach(function prepare_wp(item,idx,arr){
			let pos = new ROSLIB.Message({
				latitude: item.lat, 
				longitude: item.lng,
				altitude: 5
			});
			wp_list.push(pos)
		})

		manualMissionPath = wp_list
		showManualFlyButton()

		// layer.bindPopup('Send mission to UAV<br/><button id="hey" class="btn btn-default">Fly!</button>');

		layer._latlngs.forEach(wp => {
			if(checkDistanceTo(wp) > dist)
				dist = checkDistanceTo(wp);
		});

		if(dist < MIN_DIST_WP){
			editableLayers.addLayer(layer);
		}
    });

	document.getElementsByClassName("leaflet-draw")[0].hidden = true;


	const tj = require("@tmcw/togeojson");
	const fs = require("fs");
	const DOMParser = require("xmldom").DOMParser;
	const path = require("path");

	var uav_circle = L.circle(uav_pose, MIN_DIST_WP,{fillOpacity: 0, opacity: 0, interactive:false}).addTo(map);
	var uav_marker = L.marker(uav_pose, {icon: uav_icon, draggable: false, opacity: 0}).addTo(map);
	uav_marker.on('dragend', function (e) {
		uav_pose = uav_marker.getLatLng();
		// uav_circle.setLatLng(uav_pose);
	});

	document.getElementById('setHome').addEventListener('click', setHome);
	document.getElementById('homeNavbar').addEventListener('click', setHome);	
	document.getElementById('openMission').addEventListener('click', openMission);
	document.getElementById('openMissionNavbar').addEventListener('click', openMission);
	document.getElementById('openKMLNavbar').addEventListener('click', openKML);
	document.getElementById('openTerminal').addEventListener('click', openTerminal);
	document.getElementById('UndoMission').addEventListener('click', UndoInspectionPoints);
	document.getElementById('openAddUav').addEventListener('click', showAddUav);
	document.getElementById('openAddUavNavbar').addEventListener('click', showAddUav);
	document.getElementById('hideRosterNavbar').addEventListener('click', hideRoster);
	document.getElementById('hideRosterButton').addEventListener('click', hideRoster);
	document.getElementById('closeAddUav').addEventListener('click', showAddUav);
	document.getElementById('connectAddUav').addEventListener('click', connectAddUav);
	document.getElementById('disConnectAddUav').addEventListener('click', disConnectAddUav);
	document.getElementById('rosConnect').addEventListener('click', rosConnect, false);
	document.getElementById('rosConnectNavbar').addEventListener('click', rosConnect, false);
	document.getElementById('loadMission').addEventListener('click', loadMission);
	document.getElementById('loadMissionNavbar').addEventListener('click', loadMission);
	document.getElementById('commandMission').addEventListener('click', commandMission);
	document.getElementById('commandMissionNavbar').addEventListener('click', commandMission);
	document.getElementById('showManualMode').addEventListener('click', showManualMode);
	document.getElementById('commandManualMission').addEventListener('click', commandManualMission);

	document.getElementById('reset').addEventListener('click', reset);
	document.getElementById('resetNavbar').addEventListener('click', reset);

	document.getElementById('changeMap').addEventListener('click', changeMap);
	var openedmission = document.getElementById('openedmission');

	function clearMap() {
		mission_layers.forEach(group => {
			group.clearLayers();
		});
	}

	function setHome(){
		if(markerLayer === ""){
			map.fitBounds(SPAIN_BOUNDS);
		}else{
			map.fitBounds(markerLayer.getBounds());
		}
	}

	function openMission(){
		const { dialog } = require('electron').remote;
		const yaml = require('js-yaml');
		
		let options = {
			title: 'Pick a mission plan file',
			filters: [
				{ name: 'Yaml files', extensions: ['yaml'] },
				{ name: 'KML files', extensions: ['kml'] }
			]
		};
	
		dialog.showOpenDialog(options).then
		(
		  result => {
			if (!result.canceled)
			{
			  let paths = result.filePaths;
			  if (paths && paths.length > 0) {
				if(paths[0].split('.').pop() === 'yaml'){
					let filename = paths[0].replace(/^.*[\\\/]/, '');
					//console.log(filename);
					openedmission.innerHTML = filename;
					plan_mission = yaml.load(fs.readFileSync(paths[0], 'utf8'));
					//console.log(plan_mission)
					let plan_mission_msg = plan_mission;
					plan_mission_msg = Object.assign({id: 2}, plan_mission_msg);
					// plan_mission_msg.id = 2;					
					//console.log(plan_mission_msg);
					let yamlStr = yaml.dump(plan_mission_msg);
					fs.writeFileSync('missions/mission.yaml', yamlStr, 'utf8');
					console.log(plan_mission["mode_landing"])
					mode_landing = plan_mission["mode_landing"];
					clearMap();
					for(let n_uav = 1; n_uav <= plan_mission["uav_n"]; n_uav++){
						let latlngs = [];
						mission_layers.push(L.layerGroup([]));
						for(let n_wp = 0 ; n_wp < plan_mission["uav_"+n_uav]["wp_n"]; n_wp++){
							latlngs.push(plan_mission["uav_"+n_uav]["wp_"+n_wp])
							mission_layers[n_uav-1].addLayer(L.marker(plan_mission["uav_"+n_uav]["wp_"+n_wp],{icon: wp_blue_icon})
								.bindPopup('<b>wp_'+n_wp+' (uav_'+n_uav+')</b> <br>lat: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][0]+'<br>long: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][1]+'<br>alt: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][2]))
						};
						let polygon = L.polyline(latlngs,{color: rand_color()}).bindPopup("uav_"+n_uav);
						mission_layers[n_uav-1].addLayer(polygon).addTo(map);
						map.fitBounds(polygon.getBounds());
						let uav_idx = uav_list.findIndex(element => element.name == "uav_"+n_uav)
						// if findIndex fail uav_idx = -1 
						if(uav_idx >= 0 ){
							uav_list[uav_idx].wp_list = latlngs
						}
						showFlyButton()
					}
				}else{
					kml = new DOMParser().parseFromString(fs.readFileSync(path.resolve(__dirname, paths[0]), "utf8"));
					configure_kml();
				}
			  }
			}
		  }
		);
	}

	function configure_kml(){
		const convertedWithStyles = tj.kml(kml, { styles: true });
		// let segment_path = [];
		// function clickFeature(e) {
		// 	if(this.feature.geometry.type === "LineString"){
		// 		var dist = checkDistanceTo(e.latlng);
		// 		if(dist < MIN_DIST_WP){
		// 			let marker = L.marker(e.latlng,{icon: point_icon});
		// 			this._popup._content = '<div>Lat,Lng:'+e.latlng.lat+', '+e.latlng.lng+'</div>';
		// 			// if(inspection_init_end.length < 2){
		// 			// 	inspection_init_end.push(e.latlng)
		// 			// 	inspection_init_end_markers.push(marker)
		// 			// 	markerLayer.addLayer(marker).addTo(map)
						
		// 			// 	if(inspection_init_end.length == 2){
		// 			// 		console.log(inspection_init_end)
		// 			// 		let polygon = L.polyline(inspection_init_end,{color: rand_color()});
		// 			// 		console.log(polygon)
		// 			// 		markerLayer.addLayer(polygon).addTo(map);
		// 			// 		map.fitBounds(polygon.getBounds());
		// 			// 	}
		// 			// }
		// 		}else{
		// 			this._popup._content = '<div><img src="../assets/css/images/caution.png" style="max-width: 10%;height: auto;margin-bottom:-2px"><b>Error</b></div>'+'<br>Distance : '+dist+'m<br>Max dist: '+MIN_DIST_WP+'m</br>'
		// 		}
		// 	}
		// }

		// var selectControl = global.control = new L.Control.LineStringSelect({});
		// map.addControl(selectControl);
		
		markerLayer = global.layer= L.geoJson(convertedWithStyles, {
			pointToLayer: function (feature, latlng){
				return L
					.marker(latlng,{icon: point_icon})
					.bindPopup('<b>'+feature.properties.name +'</b>'+'<br>' +feature.properties.description + '<br>lon: ' + feature.geometry.coordinates[0] + '<br>lat: ' + feature.geometry.coordinates[1]);

			// 	switch (feature.properties.icon) {
			// 		case "http://www.oruxmaps.com/iconos/wpts_tunel.png":
			// 			return L
			// 				.marker(latlng,{icon: tunnel_icon})
			// 				.bindPopup('<b>'+feature.properties.name +'</b>'+'<br>' +feature.properties.description + '<br>lon: ' + feature.geometry.coordinates[0] + '<br>lat: ' + feature.geometry.coordinates[1]);
			// 			break;
			// 		case "http://maps.google.com/mapfiles/kml/pal4/icon25.png":
			// 			return L
			// 				.marker(latlng,{icon: point_icon})
			// 				.bindPopup('<b>'+feature.properties.name +'</b>'+'<br>' +feature.properties.description + '<br>lon: ' + feature.geometry.coordinates[0] + '<br>lat: ' + feature.geometry.coordinates[1]);
			// 			break;
			// 		default:
			// 			return L
			// 				.marker(latlng,{icon: wp_icon})
			// 				.bindPopup('<b>'+feature.properties.name +'</b>'+'<br>' +feature.properties.description + '<br>lon: ' + feature.geometry.coordinates[0] + '<br>lat: ' + feature.geometry.coordinates[1]);
			// 			break;
			// 	}
			},
			// onEachFeature(feature, layer) {
			// 	layer.on({
			// 		click: clickFeature
			// 	});
			// 	if(feature.geometry.type === "LineString"){
			// 		for (var i = 0; i < (feature.geometry.coordinates).length; i++){

			// 			if(checkDistanceTo([feature.geometry.coordinates[i][1], (feature.geometry.coordinates[i][0])]) < MIN_DIST_WP+100){
			// 				segment_path.push(feature.geometry.coordinates[i])
			// 			}
			// 		}

			// 		kml_info = '<b>'+feature.properties.name +'</b>'+'<br>' +feature.properties.description;
			// 		layer.bindPopup(kml_info);
			// 	}
			// }
		});
		markerLayer.addTo(map);

		// var geojsonFeature = {
		// 	"type": "Feature",
		// 	"properties": {},
		// 	"geometry": {
		// 		"type": "LineString",
		// 		"coordinates": segment_path // TODO Review sort operation
		// 	}
		// };

		// path_limit_layer = global.layer = L.geoJSON(geojsonFeature,{
		// 	style: function(){
		// 	  return { color: rand_color() }
		// 	}}).addTo(map);

		// path_limit_layer = path_limit_layer.getLayers()[0];
		// console.log(path_limit_layer)
		// // if(path_limit_layer._latlngs > 0){
		// 	control.enable({
		// 		feature: path_limit_layer.feature,
		// 		layer: path_limit_layer
		// 	});

		// 	control.on('selection', function() {
		// 		console.log(control)
		// 	});

			// showFlyButton()
		// }

		
		// bindPopup('Send mission to UAV<br/><button class="btn btn-default">Fly!</button>');

		// map.fitBounds(uav_circle.getBounds());
		map.fitBounds(markerLayer.getBounds());
		// // TODO: set uav real pose
		// uav_marker.setLatLng(uav_pose).setOpacity(1);
		// uav_circle.setLatLng(uav_pose).setStyle({fillOpacity: 0.2, opacity: 1});
		// console.log(uav_circle)
	}

	function openKML(){
		const { dialog } = require('electron').remote;
		const yaml = require('js-yaml');
		
		let options = {
			title: 'Pick a KML file',
			filters: [				
				{ name: 'KML files', extensions: ['kml'] }
			]
		};
	
		dialog.showOpenDialog(options).then
		(
		  result => {
			if (!result.canceled)
			{
			  let paths = result.filePaths;
			  if (paths && paths.length > 0) {
				if(paths[0].split('.').pop() === 'yaml'){
					plan_mission = yaml.load(fs.readFileSync(paths[0], 'utf8'));
					clearMap();
					for(let n_uav = 1; n_uav <= plan_mission["uav_n"]; n_uav++){
						let latlngs = [];
						mission_layers.push(L.layerGroup([]));
						for(let n_wp = 0 ; n_wp < plan_mission["uav_"+n_uav]["wp_n"]; n_wp++){
							latlngs.push(plan_mission["uav_"+n_uav]["wp_"+n_wp])
							mission_layers[n_uav-1].addLayer(L.marker(plan_mission["uav_"+n_uav]["wp_"+n_wp],{icon: wp_blue_icon})
								.bindPopup('<b>wp_'+n_wp+' (uav_'+n_uav+')</b> <br>lat: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][0]+'<br>long: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][1]+'<br>alt: '+plan_mission["uav_"+n_uav]["wp_"+n_wp][2]))
						};
						let polygon = L.polyline(latlngs,{color: rand_color()}).bindPopup("uav_"+n_uav);
						mission_layers[n_uav-1].addLayer(polygon).addTo(map);
						map.fitBounds(polygon.getBounds());
						let uav_idx = uav_list.findIndex(element => element.name == "uav_"+n_uav)
						// if findIndex fail uav_idx = -1 
						if(uav_idx >= 0 ){
							uav_list[uav_idx].wp_list = latlngs
						}
						showFlyButton()
					}
				}else{
					kml = new DOMParser().parseFromString(fs.readFileSync(path.resolve(__dirname, paths[0]), "utf8"));
					configure_kml();
				}
			  }
			}
		  }
		);
	}


	function x(){
		console.log("Esto no hace nada ahora mismo")
	}


	// ================================================================
	// Arrancando servidor websocket
	// ================================================================
	var express = require("express");
	var app = express();
	var server = require("http").Server(app);
	var io = require("socket.io")(server);

	console.log("entrando en server.js");

	//la estructura inicial de messages es el mensaje inicial que se manda al cliente al conectar por primera vez. Luego se va machacando con otra info
	var messages = 
	{
		id: 1,
		uav_id: "fuvex",
		latitude: "0",
		longitude: "0",
		altitude: "0",
		altitude_MSL: "0",
		altitude_rel: "0",
		vel_x: "0",
		vel_y: "0",
		groundspeed: "0",
		hdg: "0",
		battery: "0",
		info: "",
		frame_id: "0"
	};
	

	app.use(express.static("public"));

	app.get("/hello", function (req, res) {
	res.status(200).send("Hello Wey!");
	});

	io.on("connection", function (socket) {
	console.log("Alguien se ha conectado al Socket");
	socket.emit("messages", messages);

	socket.on("new-message", function (data) {
		if (data.id == 2) {
		console.log("peticion de mision"); // peticion de mission == dron ready para recibir mision
		if(checkUavRoster(uav_list, data.uav_id)){
			changeReady (data.uav_id);
			updateStatusWindow(data.uav_id + " asking for a mission");
		}
		// var dataYaml = readYaml(data);  //lee la info de un archivo llamada mission.yaml y la almacena en dataYaml. data no tiene uso aqui. Tendria sentido si se especifica para que uav quiere la mision
		// io.sockets.emit("messages", dataYaml); //Esto es un eco de la peticion de mision. No es necesario.
		} else if (data.id == 1) {
		console.log("recepcion de telemetria");
		if(checkUavRoster(uav_list, data.uav_id)){
			pubDataFromExt(uav_list, data);
		}
		// messages.push(data);
		messages = data;			
		io.sockets.emit("messages", messages); // Esto es un eco de la telemetria recibida. No es necesario.
		} else if (data.id == 3){
			// Mision recibida ok -> Listos para comenzar
			console.log("EXT Mission ok. Ready to launch");
			if(checkUavRoster(uav_list, data.uav_id)){
				changeReadyToFly(data);
				updateStatusWindow(data.uav_id + " is ready to fly!");
			}
		}			
		console.log(data);
		// writeYaml(data);                //escribe en un fichero data-out.yaml la info recibida por el socket				
	});
	});

	server.listen(3000, function () {
	console.log("Servidor corriendo en http://localhost:3000");		
	});

	function writeYaml(message) {
		const fs = require('fs');
		const yaml = require('js-yaml');

		let data = {message};

		let yamlStr = yaml.dump(data);
		fs.writeFileSync('missions/data-out.yaml', yamlStr, 'utf8');
	}

	function readYaml() {
		const fs = require('fs');
		const yaml = require('js-yaml');

		try {
			let fileContents = fs.readFileSync('missions/mission.yaml', 'utf8');
			let dataYaml = yaml.load(fileContents);

			console.log(dataYaml.frame_id);				
		return dataYaml;
		} catch (e) {
			console.log(e);
		}
	}
	


	function showAddUav(){
		if(document.getElementById("AddUav").style.width === "250px"){
			document.getElementById("AddUav").style.width = "0";
		}else{
			document.getElementById("AddUav").style.width = "250px";
		}
	}
	
	async function connectAddUav(){	

		var text = document.getElementById("rosConnect");

		if(text.innerHTML === "Disconnect ROS"){

			const topiclist = await getTopics2();
			console.log("despues del await")
			//console.log(topiclist)
		
			alert("Conectando Uav");
			let uav_ns = document.getElementById("UAV_NS").value;
			
			let uav_type_idx = document.getElementById("UAV_options").selectedIndex;
			let uav_type = document.getElementById("UAV_options").options[uav_type_idx].value;
			let marker_n = uav_list.length;
			let iconSelect = '../assets/css/images/' + uav_type + '-marker' + marker_n.toString() + '.png';
			uav_icon = L.icon({iconUrl: iconSelect, iconSize: [24, 24],iconAnchor: [12, 12]});
			let find_device = false;
			let repeat_device = false;
			topiclist.forEach(element =>{
				find_device = element.includes(uav_ns) || find_device;
				//console.log(element);
			});
			uav_list.forEach(element =>{
				if( element.name == uav_ns){
					repeat_device =true;
				}
			});
			if( find_device == false){ 
				alert("Dispositivo no encontrado"+uav_ns);
				return null
			}
			if( repeat_device == true){ 
				alert("Dispositivo ya se encuentra anadido"+uav_ns);
				return null
			}

			// Icon by switch
			// switch(uav_list.length) {
			// 	case 1:
			// 		uav_icon = L.icon({iconUrl: '../assets/css/images/uav-marker1.png', iconSize: [24, 24],iconAnchor: [12, 12]});
			// 	  break;
			// 	case 2:
			// 		uav_icon = L.icon({iconUrl: '../assets/css/images/uav-marker2.png', iconSize: [24, 24],iconAnchor: [12, 12]});
			// 	  break;
			// 	default:
			// 		uav_icon = L.icon({iconUrl: '../assets/css/images/uav-marker.png', iconSize: [24, 24],iconAnchor: [12, 12]});
			//   }			
			
			let uav_marker = L.rotatedMarker(uav_pose, {icon: uav_icon, draggable: false, opacity: 0}, {
				rotationOrigin: 'center center'
			  }).addTo(map)
			  	.bindPopup(uav_ns)
    			.openPopup();
			let uavAdded = { name : uav_ns, type : uav_type, pose : [] , marker : uav_marker, watch_bound : true, wp_list : [] , listener : "", listener_hdg : "", listener_alt : "", listener_vel : "", listener_bat : "",bag : false};
			uav_list.push(uavAdded);
			let cur_uav_idx = uav_list.length-1;

			//Insertar fila en la tabla de uavs
			// Find a <table> element with id="uavTable":
			var tabla = document.getElementById("uavTable");

			// Create an empty <tr> element and add it to the next position of the table:
			if(uav_list.length==1){
				document.getElementById("uavTable").deleteRow(1);
			}
			var fila = tabla.insertRow(cur_uav_idx+1);
			fila.setAttribute("id", uav_ns);
			
			// Insert new cells (<td> elements) at every position of the "new" <tr> element:
			var cell1 = fila.insertCell(0);
			var cell2 = fila.insertCell(1);
			var cell3 = fila.insertCell(2);
			var cell4 = fila.insertCell(3);
			var cell5 = fila.insertCell(4);
			var cell6 = fila.insertCell(5);
			cell6.innerHTML = "Waiting uav"
			
			var buttonReady = document.createElement("button");
			buttonReady.setAttribute("class", "btn btn-default icon-front");
			buttonReady.setAttribute("id", "Ready"+uav_ns);
			buttonReady.innerHTML = "Not Ready";
			buttonReady.style.marginLeft = 0;
			buttonReady.addEventListener("click", function () {
				changeReady(uav_ns);				
			  });			
			cell5.appendChild(buttonReady);

			// Add some text to the new cells:
						
			cell1.innerHTML = uavAdded.name;
			cell1.float = "right";
			// Add button & image after uav name
			var buttonMarker = document.createElement("button");
			buttonMarker.onclick = function () {
				GoToUavPosition(map, cur_uav_idx);				
			  };
			buttonMarker.setAttribute("class", "uav-buttonMarker");
			cell1.appendChild(buttonMarker);
						
			// Subscribing
			// DJI 
			if(uav_type == "dji"){

				var buttonLoad = document.createElement("button");
				buttonLoad.setAttribute("class", "btn btn-default icon-front icon icon-upload");
				// buttonLoad.style.marginLeft = '25%';
				buttonLoad.addEventListener("click", function () {
					loadMissionToUAV(uav_ns);				
				  });			
				cell5.appendChild(buttonLoad);
				var buttonFlyToUav = document.createElement("button");
				buttonFlyToUav.setAttribute("class", "btn btn-default icon-front");
				buttonFlyToUav.innerHTML = "Fly!";			
				buttonFlyToUav.addEventListener("click", function () {
				commandMissionToUAV(uav_ns);				
			  		});			
				cell5.appendChild(buttonFlyToUav);

				uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/dji_osdk_ros/rtk_position',
					messageType : 'sensor_msgs/NavSatFix'
				});

				uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/dji_osdk_ros/rtk_yaw',
					messageType : 'std_msgs/Int16'
				});

				uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/dji_osdk_ros/velocity',
					messageType : 'geometry_msgs/Vector3Stamped'
				});

				uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/dji_osdk_ros/battery_state',
					messageType : 'sensor_msgs/BatteryState'
				});

				uav_list[cur_uav_idx].listener.subscribe(function(message) {
					// console.log('Received message on ' + listen_gps_pos.name + ': ' + message.latitude);
					uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
					uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);
					
					var showData = document.getElementById(uav_ns).cells;
  					showData[1].innerHTML = (message.altitude - 493.5).toFixed(2);
					// Fill info cell. Lo hacemos aqui para que se actualice cada vez que recibimos un mensaje en este topic
					var showData = document.getElementById(uav_ns).cells;
					// podemos poner aqui una condicion para que solo muestre connected al principio (por ejemplo: si innerHTML == "Waiting uav", o no hacerlo si es == "Ready to launch")
			  		showData[5].innerHTML = "Connected";    
					
					var buttonMarkerSelect = '<img src="../assets/css/images/dji-marker'+marker_n.toString()+'.png " width="24" height="24" />'; //class="rotateimg180"
					buttonMarker.innerHTML = buttonMarkerSelect;
										
					if (uav_list[cur_uav_idx].watch_bound){
						uav_list[cur_uav_idx].marker.setOpacity(1);
						map.fitBounds(L.latLngBounds([ uav_list[cur_uav_idx].marker.getLatLng() ]));  
						uav_list[cur_uav_idx].watch_bound = false
					}
				});

				uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
					uav_list[cur_uav_idx].marker.setRotationAngle(message.data+90);
					var markerRotation = message.data+90;
					buttonMarker.style.transform = 'rotate(' + markerRotation.toString() + 'deg)';
				});				
				
				uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[2].innerHTML = Math.sqrt(Math.pow(message.vector.x,2) + Math.pow(message.vector.y,2)).toFixed(2);
				});				
				
				uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[3].innerHTML = message.percentage + "%";
				});
				
			}else if(uav_type == "px4"){
				// PX4

				var buttonLoad = document.createElement("button");
				buttonLoad.setAttribute("class", "btn btn-default icon-front icon icon-upload");
				// buttonLoad.style.marginLeft = '25%';
				buttonLoad.addEventListener("click", function () {
					loadMissionToUAV(uav_ns);				
				  });			
				cell5.appendChild(buttonLoad);
				var buttonFlyToUav = document.createElement("button");
				buttonFlyToUav.setAttribute("class", "btn btn-default icon-front");
				buttonFlyToUav.innerHTML = "Fly!";			
				buttonFlyToUav.addEventListener("click", function () {
				commandMissionToUAV(uav_ns);				
			  		});			
				cell5.appendChild(buttonFlyToUav);


				uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/global_position/global',
					messageType : 'sensor_msgs/NavSatFix'
				});

				uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/global_position/compass_hdg',
					messageType : 'std_msgs/Float64'
				});

				uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/local_position/velocity_local',
					messageType : 'geometry_msgs/TwistStamped'
				});

				uav_list[cur_uav_idx].listener_alt = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/altitude',
					messageType : 'mavros_msgs/Altitude'
				});

				uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/battery',
					messageType : 'sensor_msgs/BatteryState'
				});

				uav_list[cur_uav_idx].listener.subscribe(function(message) {
					uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
					uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);

					// Fill info cell. Lo hacemos aqui para que se actualice cada vez que recibimos un mensaje en este topic
					var showData = document.getElementById(uav_ns).cells;
					// podemos poner aqui una condicion para que solo muestre connected al principio (por ejemplo: si innerHTML == "Waiting uav", o no hacerlo si es == "Ready to launch")
			  		showData[5].innerHTML = "Connected";

					var buttonMarkerSelect = '<img src="../assets/css/images/px4-marker'+marker_n.toString()+'.png " width="24" height="24" />';
					buttonMarker.innerHTML = buttonMarkerSelect;
										
					if (uav_list[cur_uav_idx].watch_bound){
						uav_list[cur_uav_idx].marker.setOpacity(1);
						map.fitBounds(L.latLngBounds([ uav_list[cur_uav_idx].marker.getLatLng() ]));  
						uav_list[cur_uav_idx].watch_bound = false
					}
				});

				uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
					uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
				});

				uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[1].innerHTML = (message.relative).toFixed(2);
					  			
				});

				uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
				});

				uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
				});
			} else {

				//EXT (FUVEX)

				var buttonLoad = document.createElement("button");
				buttonLoad.setAttribute("class", "btn btn-default icon-front icon icon-upload");
				// buttonLoad.style.marginLeft = '25%';
				buttonLoad.addEventListener("click", function () {
					loadMissionToExt(uav_ns);				
				});			
				cell5.appendChild(buttonLoad);
				var buttonFlyToUav = document.createElement("button");
				buttonFlyToUav.setAttribute("class", "btn btn-default icon-front");
				buttonFlyToUav.setAttribute("id", "ReadyToFly"+uav_ns);
				// buttonFlyToUav.style.marginLeft = '50%';
				buttonFlyToUav.innerHTML = "X";			
				buttonFlyToUav.addEventListener("click", function () {
					commandMissionToEXT(uav_ns);				
			  	});			
				cell5.appendChild(buttonFlyToUav);


				uav_list[cur_uav_idx].listener = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/global_position/global',
					messageType : 'sensor_msgs/NavSatFix'
				});

				uav_list[cur_uav_idx].listener_hdg = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/global_position/compass_hdg',
					messageType : 'std_msgs/Float64'
				});

				uav_list[cur_uav_idx].listener_vel = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/local_position/velocity_local',
					messageType : 'geometry_msgs/TwistStamped'
				});

				uav_list[cur_uav_idx].listener_alt = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/altitude',
					messageType : 'mavros_msgs/Altitude'
				});

				uav_list[cur_uav_idx].listener_bat = new ROSLIB.Topic({
					ros : ros,
					name : uav_ns+'/mavros/battery',
					messageType : 'sensor_msgs/BatteryState'
				});

				uav_list[cur_uav_idx].listener.subscribe(function(message) {
					uav_list[cur_uav_idx].pose = [message.latitude, message.longitude];
					uav_list[cur_uav_idx].marker.setLatLng(uav_list[cur_uav_idx].pose);

					var buttonMarkerSelect = '<img src="../assets/css/images/ext-marker'+marker_n.toString()+'.png " width="24" height="24" />';
					buttonMarker.innerHTML = buttonMarkerSelect;
					
					if (uav_list[cur_uav_idx].watch_bound){
						uav_list[cur_uav_idx].marker.setOpacity(1);
						map.fitBounds(L.latLngBounds([ uav_list[cur_uav_idx].marker.getLatLng() ]));  
						uav_list[cur_uav_idx].watch_bound = false
					}
				});

				uav_list[cur_uav_idx].listener_hdg.subscribe(function(message) {
					uav_list[cur_uav_idx].marker.setRotationAngle(message.data)
				});

				uav_list[cur_uav_idx].listener_alt.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[1].innerHTML = (message.relative).toFixed(2);										
				});

				uav_list[cur_uav_idx].listener_vel.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[2].innerHTML = Math.sqrt(Math.pow(message.twist.linear.x,2) + Math.pow(message.twist.linear.y,2)).toFixed(2);
				});

				uav_list[cur_uav_idx].listener_bat.subscribe(function(message) {
					var showData = document.getElementById(uav_ns).cells;
  					showData[3].innerHTML = (message.percentage*100).toFixed(0) + "%";
				});
			}

			console.log("\nLa Lista de uav's es: ");
			uav_list.forEach(function(uav, indice, array) {
				console.log(uav, indice);
			})

			// alert("Total de drones conectados: "+uav_list.length+"\n Dron añadido a la lista:\n\nNombre: "+uavAdded.name +"\n Tipo: "+ uavAdded.type);
			updateStatusWindow(uavAdded.name + " added. Type: "+ uavAdded.type);

		// showAddUav();
		}else{
			alert("\nRos no está conectado.\n\n Por favor conéctelo primero.")

		} // if ros connected


	}

})

	function rosConnect(){
		var text = document.getElementById("rosConnect");

		if(text.innerHTML === "Connect ROS"){
			// Connecting to ROS
			ros = new ROSLIB.Ros({
				url : 'ws://localhost:9090'
			});

			ros.on('connection', function() {
				console.log('Connected to websocket server.');
			});

			ros.on('error', function(error) {
				console.log('Error connecting to websocket server: ', error);
			});

			ros.on('close', function() {
				console.log('Connection to websocket server closed.');
			});
			
			text.innerHTML = "Disconnect ROS";
			updateStatusWindow("Rosbridge connected");
		}else{

			//killallbag()

			// listen_gps_pos.unsubscribe()
			uav_marker.setOpacity(0);
			// uav_circle.setStyle({fillOpacity: 0, opacity: 0});
			// circle_set = false
			uav_pose = [0, 0]

			for (var i = 0; i < uav_list.length; i++) {
				uav_list[i].listener.unsubscribe();
				uav_list[i].marker.setOpacity(0);				
				uav_list[i].pose = [0, 0];
				document.getElementById("uavTable").deleteRow(uav_list.length-i);
			}

			uav_list = [];

			console.log("\nLa Lista de uav está vacía: ");
			// uav_list.forEach(function(elemento, indice, array) {
			// console.log(elemento, indice);
			// })

			

			ros.close()

			text.innerHTML = "Connect ROS";
			updateStatusWindow("Rosbridge disconnected");
		}
	}

	function disConnectAddUav(){
		let cur_uav_idx = uav_list.length-1;
		if (uav_list.length != 0){
			uav_list[cur_uav_idx].listener.unsubscribe();
			uav_list[cur_uav_idx].listener_hdg.unsubscribe();
			uav_list[cur_uav_idx].listener_vel.unsubscribe();
			uav_list[cur_uav_idx].listener_bat.unsubscribe();
			if( uav_list[cur_uav_idx].bag)
			{
				//Rosbagtestend(uav_list[cur_uav_idx].name)
			}
			uav_list[cur_uav_idx].marker.setOpacity(0);
			uav_list[cur_uav_idx].pose = [];
			if (uav_list[cur_uav_idx].type != "dji"){
				uav_list[cur_uav_idx].listener_alt.unsubscribe();
			}
			var tabla = document.getElementById("uavTable");
			document.getElementById("uavTable").deleteRow(uav_list.length);
			if(uav_list.length==1){
				var fila = tabla.insertRow(cur_uav_idx+1);
			var cell1 = fila.insertCell(0);
				cell1.innerHTML = "No uavs";
				// var cell2 = fila.insertCell(1);
				// var cell3 = fila.insertCell(2);
				// var cell4 = fila.insertCell(3);
			}
			
			uav_list.pop();
			alert("Último dron eliminado de la lista");
			if (uav_list.length != 0){
				console.log("\nLa Lista de uav actualizada es: ");
				uav_list.forEach(function(elemento, indice, array) {
				console.log(elemento, indice);
				})
			}else{
				alert("No quedan drones en la lista");
			}

		}else{
			alert("No hay drones conectados");
		}
	}


	function loadMission(){ 
		// console.log(control.getSelection())
		// let path_wp = control.getSelection()
		let cur_roster = []
		let cur_ns = ""
		uav_list.forEach(function prepare_wp(item,idx,arr){
			cur_ns = item.name
			cur_roster.push(cur_ns);
			if (item.type != "ext"){
				let wp_command = []
					item.wp_list.forEach(function prepare_wp(item,idx,arr){
					let pos = new ROSLIB.Message({
						latitude: item[0], 
						longitude: item[1],
						altitude: item[2]  //Creo que aqui si añadimos otro campo, cuando le des arriba a load mission se subiria bien añadiendo yaw, por ej. Load missiontouav creo que es para subirle la mision de forma individual a cada uno, desde la tabla
					});

					wp_command.push(pos)
				})

				
				if(item.type == "dji"){
					var missionClient = new ROSLIB.Service({
						ros : ros,
						name : cur_ns + '/dji_control/configure_mission',
						serviceType : 'aerialcore_common/ConfigMission'
					});
				}else{
					var missionClient = new ROSLIB.Service({
						ros : ros,
						name : cur_ns + '/mission/new',
						serviceType : 'aerialcore_common/ConfigMission'
					});
				}

				var request = new ROSLIB.ServiceRequest({
					type : "waypoint",
					waypoint: wp_command,
					radius : 0,
					maxVel:	3,
					idleVel: 1.8,
					yawMode: 0,
					traceMode: 0,
					finishAction: mode_landing
				});
				console.log(request)

				missionClient.callService(request, function(result) {
					console.log('load mission'+ missionClient.name+': '+result.success);
						if(result.success){
							updateStatusWindow("Load mission to:" + cur_roster + " ok");
						} else{
							updateStatusWindow("Load mission to:" + cur_roster + " fail");
						}
				}, function(result) {
					console.log('Error:'
					+ result);
					alert('Error:'
					+ result);
				});
				console.log('loading mision to'+cur_ns);			
			} else{
				// if (item.type == ext)
				// Si mandamos la mision individual para cada uav, hay que hacerlo por este camino, y descomentar la linea siguiente.
				loadMissionToExt(cur_ns);
				// var info = "Loading Mission";
				// updateInfoCell(cur_ns, info);
			}
		
		})
		// La linea siguiente tiene en cuenta que se mandan las misiones de todos los uavs EXT a la vez. Si no fuese así, habría que comentar la linea siguiente.
		// loadMissionToExt(cur_ns);
		let flyButton = document.getElementById("commandMission");
		if(flyButton.style.cssText == "visibility: none;"){
			flyButton.style.cssText = "visibility: hidden;"
		}else{
			flyButton.style.cssText = "visibility: none;"
		}
		updateStatusWindow("Loadding mission to: " + cur_roster);
	}

	function commandMission(){
		var r = confirm("Comand mission?");
		if (r == true) {
			let cur_roster = []
			uav_list.forEach(function prepare_wp(uav,idx,arr){
				cur_roster.push(uav.name);
				let missionClient;
				if (uav.type != "ext"){
					if(uav.type == "dji"){
						missionClient = new ROSLIB.Service({
							ros : ros,
							name : uav.name+'/dji_control/start_mission',
							serviceType : 'std_srvs/SetBool'
						});

					}else{
						missionClient = new ROSLIB.Service({
							ros : ros,
							name : uav.name+'/mission/start_stop',
							serviceType : 'std_srvs/SetBool'
						});
					}
				
					let request = new ROSLIB.ServiceRequest({data: true});

					missionClient.callService(request, function(result) {
						console.log(result.message);
						if(result.success){
							updateStatusWindow("Start mission:" + cur_roster + " ok");
						} else{
							updateStatusWindow("Start mission:" + cur_roster + " FAIL!");
						}
					});
				} else {
					console.log(uav.name);
					commandMissionToEXT(uav.name);
				}
			});
			updateStatusWindow("Commanding mission to: " + cur_roster);
		} else {
			console.log("Mission canceled")
		}
	}
	
	function loadMissionToUAV(ns){
		// console.log(control.getSelection())
		// let path_wp = control.getSelection()
		let uav_idx = uav_list.findIndex(element => element.name == ns)
		let wp_command = []
		uav_list[uav_idx].wp_list.forEach(function prepare_wp(item,idx,arr){
			let pos = new ROSLIB.Message({
				latitude: item[0], 
				longitude: item[1],
				altitude: item[2]
			});

			wp_command.push(pos)
		})

		if(uav_list[uav_idx].type == "dji"){
			var missionClient = new ROSLIB.Service({
				ros : ros,
				name : ns + '/dji_control/configure_mission',
				serviceType : 'aerialcore_common/ConfigMission'
			});
		}else{
			var missionClient = new ROSLIB.Service({
				ros : ros,
				name : ns + '/mission/new',
				serviceType : 'aerialcore_common/ConfigMission'
			});
		}

		var request = new ROSLIB.ServiceRequest({
			type : "waypoint",
			waypoint: wp_command,
			radius : 0,
			maxVel:	3,
			idleVel: 2,
			yawMode: 0,
			traceMode: 0,
			finishAction: mode_landing
		});

		missionClient.callService(request, function(result) {
			console.log('Result for service call on '
			+ missionClient.name
			+ ': '
			+ result.success);
		}, function(result) {
			console.log('Error:'
			+ result);
			alert('Error:'
			+ result);
		});

		// let flyButton = document.getElementById("commandMission");
		// if(flyButton.style.cssText == "visibility: none;"){
		// 	flyButton.style.cssText = "visibility: hidden;"
		// }else{
		// 	flyButton.style.cssText = "visibility: none;"
		// }
		updateStatusWindow("Loadding mission to: " + ns);
	}

	function loadMissionToExt(uav_ns){
		//actualmente no se criba por el id del uav, pero se podria. Se envía toda la mision, con todas los uavs EXT
		var buttonReady = document.getElementById("Ready"+uav_ns);
		if (buttonReady.innerHTML == "Ready")	{
			//Carga el archivo yaml de mision
			var dataYaml = readYaml();
			//emite la mision (broadcasting)
			io.sockets.emit("messages", dataYaml); 
			console.log("Loading mission to EXT"+uav_ns);
			var info = "Loading Mission";
			updateInfoCell(uav_ns, info);
			updateStatusWindow("Loadding mission to: " + uav_ns);
		} else {
			console.log(uav_ns+' is not ready to receive the mission');
			updateStatusWindow(uav_ns+' is not ready');
		}
		
	}


	function commandMissionToUAV(ns){
		var r = confirm("Comand mission to "+ns+"?");
		if (r == true) {
			let uav_idx = uav_list.findIndex(element => element.name == ns)
			let missionClient;
			if(uav_list[uav_idx].type == "dji"){
				missionClient = new ROSLIB.Service({
					ros : ros,
					name : ns+'/dji_control/start_mission',
					serviceType : 'std_srvs/SetBool'
				});

			}else{
				missionClient = new ROSLIB.Service({
					ros : ros,
					name : ns+'/mission/start_stop',
					serviceType : 'std_srvs/SetBool'
				});
			}
			let request = new ROSLIB.ServiceRequest({data: true});

			missionClient.callService(request, function(result) {
			console.log(result.message);
			});
			updateStatusWindow("Commanding mission to: " + ns);

			
			//Rosbagtestbegin(uav_list[uav_idx].name);

		} else {
			console.log("Mission canceled")
		}
	}


	function commandMissionToEXT(uav_ns){
		var button = document.getElementById("ReadyToFly"+uav_ns);
		if(button.innerHTML == "Fly!"){		
		console.log("Command Mission to EXT"+uav_ns);
		// Hacemos la orden individual al uav que lo haya solicitado
		
		var message = {
		  id: 4,
		  uav_id: uav_ns,
		  info: "Mission Commanded"
		};
		
		io.sockets.emit("messages", message);
		// return false;
		var showData = document.getElementById(uav_ns).cells;
  		showData[5].innerHTML = "Mission Commanded";
		updateStatusWindow("Commanding mission to: " + uav_ns);

		//Rosbagtestbegin(uav_ns)

		} else {
			console.log("Mission is not loaded yet on "+uav_ns);
			updateStatusWindow("Mission is not loaded yet on "+uav_ns);
		}
	}

	function pubDataFromExt(uav_list, data){
		console.log("Received! Telemetry from EXT");		   
		// Topic debe estar creado. Si no, entonces hay algun error
		let uav_idx = uav_list.findIndex(element => element.name == data.uav_id);
		// if findIndex fail --> uav_idx = -1 
		if(uav_idx >= 0 ){
		// Create ROSLIB.Message for each data field		
			var pose = new ROSLIB.Message({
				latitude: Number(data.latitude),
				longitude: Number(data.longitude)				
			});
			var hdg = new ROSLIB.Message({
				data: Number(data.hdg)
			});
			var vel = new ROSLIB.Message({
				twist : {
					linear : {
						x : Number(data.vel_x),
						y : Number(data.vel_y),
						// z : 0.0
						}
						// ,angular : {
						// x : -0.1,
						// y : -0.2,
						// z : -0.3
						// }
				}				
			});
			var alt = new ROSLIB.Message({
				relative: Number(data.altitude_rel)
			});
			var bat = new ROSLIB.Message({
				percentage: Number(data.battery)
			});
			// publish data telemetry messages
			uav_list[uav_idx].listener.publish(pose);
			uav_list[uav_idx].listener_hdg.publish(hdg);
			uav_list[uav_idx].listener_vel.publish(vel);
			uav_list[uav_idx].listener_alt.publish(alt);
			uav_list[uav_idx].listener_bat.publish(bat);
			// fill info cell at uav's table
			var showData = document.getElementById(data.uav_id).cells;
  			showData[5].innerHTML = data.info;

			
	
		} else{
			console.log(data.uav_id+"UAV no inicializado");
		}
	}

	function commandManualMission() {

		var missionClient = new ROSLIB.Service({
			ros : ros,
			name : '/dji_control/configure_mission',
			serviceType : 'inspector/ConfigMission'
		});

		var request = new ROSLIB.ServiceRequest({
			type : "waypoint",
			waypoint: manualMissionPath,
			radius : 0,
			maxVel:	2,
			idleVel: 1,
			yawMode: 0,
			traceMode: 0,
			finishAction: mode_landing
		});
		
		missionClient.callService(request, function(result) {
			console.log('Result for service call on '
			+ missionClient.name
			+ ': '
			+ result.success);
		}, function(result) {
			console.log('Error:'
			+ result);
		});
	}


	function showManualMode(){
		let manualTools = document.getElementsByClassName("leaflet-draw");
		manualTools[0].hidden = !manualTools[0].hidden;
	}

	function showFlyButton(){
		let flyButton = document.getElementById("loadMission");
		if(flyButton.style.cssText == "visibility: none;"){
			flyButton.style.cssText = "visibility: hidden;"
		}else{
			flyButton.style.cssText = "visibility: none;"
		}
	}

	function showManualFlyButton(){
		let flyButton = document.getElementById("commandManualMission");
		if(flyButton.style.cssText == "visibility: none;"){
			flyButton.style.cssText = "visibility: hidden;"
		}else{
			flyButton.style.cssText = "visibility: none;"
		}
	}

	function reset(){
		if(markerLayer){
			map.removeLayer(markerLayer)
			if(path_limit_layer.length > 0){
				control.disable()
				map.removeLayer(path_limit_layer)
			}
			kml = "",
			kml_info = "",
			markerLayer = "",
			plan_mission = "";
			uav_marker.setOpacity(0);
			uav_circle.setStyle({fillOpacity: 0, opacity: 0});
		}

		mission_layers.forEach(group => {
			group.clearLayers();
		});

		setHome();	
	}

	function changeMap(){
		const { dialog } = require('electron').remote;
		const yaml = require('js-yaml');
		
		let options = {
			title: 'Select local map tiles folder',
			properties: ['openDirectory']
		};
	
		dialog.showOpenDialog(options).then
		(
		  result => {
			if (!result.canceled)
			{
			  let path = result.filePaths;
				console.log(path)

			  if (path && path.length > 0) {
				baseMap.setUrl(path+"/{z}/{x}/{y}.png",true)
			  }
			}
		  }
		);
	}


function checkDistanceTo() {
	return L.latLng(uav_pose).distanceTo(arguments[0]).toFixed(2);
}

function rand_color(){
	let r = Math.floor(Math.random() * 255);
	let g = Math.floor(Math.random() * 255);
	let b = Math.floor(Math.random() * 255);
	return "rgb("+r+" ,"+g+","+ b+")";
}

function UndoInspectionPoints(){
	if(inspection_init_end_markers.length > 0){
		markerLayer.removeLayer(inspection_init_end_markers[inspection_init_end_markers.length-1]);
		inspection_init_end.pop();		
		inspection_init_end_markers.pop();		
	}
}

function GoToUavPosition(map, cur_uav_idx){
	console.log("estas pinchando el iconito");
	map.fitBounds(L.latLngBounds([ uav_list[cur_uav_idx].marker.getLatLng() ]));  
	uav_list[cur_uav_idx].watch_bound = false;
}

function changeReady(uav_ns){
	var button = document.getElementById("Ready"+uav_ns);
	if(button.innerHTML == "Ready"){
		var r = confirm("Change to Not Ready");
		if (r == true) {
			button.innerHTML = "Not Ready";
	    } else{
			console.log("UAV still Ready");
		}
	}else{
	button.innerHTML = "Ready";
	// fila.cell5.buttonReady.innerHTML = "Ready";
	var info = "Mission requested";
	updateInfoCell(uav_ns, info);
	}
}

function changeReadyToFly(data){
	var button = document.getElementById("ReadyToFly"+data.uav_id);
	if(button.innerHTML == "Fly!"){
		console.log("Ready to Fly previously received");
		// do nothing
		// var r = confirm("Fly Mission?");
		// if (r == true) {
		// 	button.innerHTML = "Not Ready";
	    // } else{
		// 	console.log("UAV still Ready");
		// }
	}else{
	var buttonReady = document.getElementById("Ready"+data.uav_id);
	if (buttonReady.innerHTML == "Ready")	{
		button.innerHTML = "Fly!";
		console.log(data.uav_id+' Ready To Fly');
		var info = "Ready to Fly";
		updateInfoCell(data.uav_id, info);		
	} else {
		console.log("Loaded Mission is not accepted yet");
	}
	
	}
}

function updateInfoCell (uav_ns, info) {
	var showData = document.getElementById(uav_ns).cells;
  	showData[5].innerHTML = info;
}

function updateStatusWindow (text) {
	var statusLenght;
	var status = document.getElementById("statusWindow");
	statusLenght = statusLog.push("\n"+text);
	if (statusLenght > 7){
		statusLog.shift();
	}
	// var newinfo = {
	// 	info: text
	// }
	// logMessages.push(newinfo);	
  	// status.innerHTML = logMessages;
	// status.style.color = "red";
	status.innerHTML = statusLog;
	
}

/* Toggle between showing and hiding the uav Roster when the user clicks on the menu icon */
function hideRoster() {
	var x = document.getElementById("uavTable");
	if (x.style.display === "block") {
	  x.style.display = "none";
	} else {
	  x.style.display = "block";
	}
}

function hideStatusWindow() {
var x = document.getElementById("statusWindow");
	if (x.style.display === "block") {
		x.style.display = "none";
	} else {
		x.style.display = "block";
	}
}

function checkUavRoster(uav_list,uav_id){
var result;
let uav_idx = uav_list.findIndex(element => element.name == uav_id);
// if findIndex fail --> uav_idx = -1 
	if(uav_idx >= 0 ){
		result = 1;
	}else {
		result = 0;
		console.log(uav_id+"UAV no inicializado");
		updateStatusWindow(uav_id + " No inicializado");
	}
	return result;
} 
function getTopics() {
    var topicsClient = new ROSLIB.Service({
    ros : ros,
    name : '/rosapi/topics',
    serviceType : 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();

    topicsClient.callService(request, function(result) {
    console.log("Getting topicsss...");
    console.log(result.topics);
	listoftopic = result.topics;
    });
};
const getTopics2 = () => {
    var topicsClient = new ROSLIB.Service({
    ros : ros,
    name : '/rosapi/topics',
    serviceType : 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();

	return new Promise((resolve,rejects) => {
		topicsClient.callService(request, function(result) {
			resolve(result.topics);
		});
	});
};

//------------------------------------------
function Rosbagtestbegin(uav_name){
	let id_bag = 0,i=0
	uav_list.forEach(element =>{
		if(element.name ==uav_name){
			id_bag = i;
		}
		i=i+1;
	});
	uav_list[id_bag].bag = true;

	console.log("begin ros bag "+uav_name+" "+	dt.format("Y-m-d-H-mm"))
	exec('rosbag record -O ./bag/'+uav_name+'_'+dt.format("Y-m-d-H-mm")+'.bag  -e "/'+uav_name+'/dji_osdk_ros/(.*)" __name:=node_bag_'+uav_name+' &', (error, stdout, stderr) => {
		if (error) {
			console.log(`[ERROR] openCashDrawer: ${error.message}`);
			return;
		}
		
		if (stderr) {
			console.log(`[STDERROR] openCashDrawer: ${stderr}`);
			return;
		}
	
		console.log(`openCashDrawer: ${stdout}`); // Output response from the terminal
		});
}

function Rosbagtestend(uav_name){
	console.log("end ros bag "+uav_name)
	exec('rosnode kill /node_bag_'+uav_name, (error, stdout, stderr) => {
		if (error) {
			console.log(`[ERROR] openCashDrawer: ${error.message}`);
			return;
		}
		
		if (stderr) {
			console.log(`[STDERROR] openCashDrawer: ${stderr}`);
			return;
		}
	
		console.log(`openCashDrawer: ${stdout}`); // Output response from the terminal
		});
}

function killallbag() {

	uav_list.forEach(element =>{
		if( element.bag ){
			Rosbagtestend(element.name);
		}
	});
}

module.exports = function exporkillallbag () {
	//killallbag()
};
  