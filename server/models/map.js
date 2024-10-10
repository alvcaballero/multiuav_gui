import * as turf from '@turf/turf';
import { NoElevation } from '../config/config.js';
import { now } from 'sequelize/lib/utils';

export class mapModel {
  static async ApiElevation(LocationList) {
    let myresponse = {};
    let divLocationList = [];
    let maxAPIlocation = 99;
    if (LocationList.length > maxAPIlocation) {
    }
    for (let i = 0; i < LocationList.length; i = i + maxAPIlocation) {
      if (LocationList.length - i < maxAPIlocation) {
        divLocationList.push(LocationList.slice(i));
        break;
      }
      divLocationList.push(LocationList.slice(i, i + maxAPIlocation));
    }

    let stringLocationList = divLocationList.map((list) => list.map((waypoint) => waypoint.join(',')).join('|'));
    if (NoElevation) {
      if (!myresponse.hasOwnProperty('results')) {
        myresponse = { status: 'warning', results: [] };
      }
      console.log('Disable Elevation API');
      for (let i = 0; i < stringLocationList.length; i = i + 1) {
        divLocationList[i].map((element) => {
          myresponse.results.push({
            location: { lat: element[0], lng: element[1] },
            elevation: 0,
          });
        });
        return myresponse;
      }
    }
    console.log('response elevation----');
    for (let i = 0; i < stringLocationList.length; i = i + 1) {
      await fetch(`https://api.opentopodata.org/v1/eudem25m?locations=${stringLocationList[i]}`)
        .then((response) => response.json())
        .catch(function (error) {
          console.log('Problem in  Fetch:' + error.message);
          if (!myresponse.hasOwnProperty('results')) {
            myresponse = { status: 'warning', results: [] };
          }
          divLocationList[i].map((element) => {
            myresponse.results.push({
              location: { lat: element[0], lng: element[1] },
              elevation: 0,
            });
          });
          return myresponse;
        })
        .then((body) => {
          //console.log(body);
          if (myresponse.hasOwnProperty('results') && myresponse.results.length > 0) {
            body.results.map((element) => {
              myresponse.results.push(element);
            });
          } else {
            myresponse = body;
          }
        });
    }

    return myresponse;
  }

  static async calcElevation(routes) {
    console.log('using ---- elevation');
    let listpoint = routes;
    //console.log(listpoint);
    let wpaltitude = [];
    let listwaypoint = [];
    let status = true;
    if (listpoint.length > 0) {
      listpoint.forEach((route, index_rt, array_rt) => {
        let acumulative = [];
        let lastindex = 0;
        let lastdist = 0;
        wpaltitude.push([]);
        route.forEach((wp, index, array) => {
          let lineLength = 0;
          let altitud = wp[2];
          acumulative.push(wp);
          if (index != 0) {
            let linestring = turf.lineString(acumulative);
            lineLength = turf.length(linestring, { units: 'meters' });
            //medir que distancia sea mayor
            let otherline = turf.lineString([wp, array[lastindex]]);
            let distbetweenwp = turf.length(otherline, { units: 'meters' });
            //console.log('distancia puntos ' + distbetweenwp);
            if (distbetweenwp > 200) {
              //console.log('mayor a 200 metros');
              //funcion de slice and add to  //altitud = -1;
              let steps = Math.floor(distbetweenwp / 200) + 1;
              let newpoints = this.divideLineIntoPoints([array[lastindex], wp], steps, distbetweenwp);
              newpoints.map((nwp) => {
                listwaypoint.push([nwp.lat, nwp.lon]);
                let nwpdist = +lastdist.toFixed(1) + +nwp.dist.toFixed(1);
                //console.log('lastdist ' + lastdist + ' caldist ' + nwp.dist.toFixed(1));

                wpaltitude[index_rt].push({
                  length: nwpdist,
                  uav: null,
                });
              });
            }
          }
          listwaypoint.push([wp[0], wp[1]]);
          wpaltitude[index_rt].push({
            length: Number(lineLength.toFixed(1)),
            uav: altitud,
          });
          lastindex = index;
          lastdist = Number(lineLength.toFixed(1));
        });
      });
      //console.log(wpaltitude);
      //console.log(listwaypoint);

      let elevationprofile = await this.ApiElevation(listwaypoint);
      //anadir elevacion profile
      //console.log(elevationprofile);
      let auxcount = 0;
      let initElevationIndex = 0;

      try {
        for (let index_rt = 0; index_rt < wpaltitude.length; index_rt++) {
          let wp_count = 0;
          for (let index = 0; index < wpaltitude[index_rt].length; index++) {
            wpaltitude[index_rt][index]['elevation'] = Number(elevationprofile.results[auxcount].elevation).toFixed(1);
            wpaltitude[index_rt][index]['lat'] = +elevationprofile.results[auxcount].location.lat;
            wpaltitude[index_rt][index]['lng'] = +elevationprofile.results[auxcount].location.lng;
            console.log(
              'elevation',
              wpaltitude[index_rt][index]['elevation'],
              'lat',
              wpaltitude[index_rt][index]['lat'],
              'lng',
              wpaltitude[index_rt][index]['lng']
            );

            if (index == 0) {
              initElevationIndex = auxcount;
            }

            wpaltitude[index_rt][index]['rt'] = index_rt;
            if (wpaltitude[index_rt][index]['uav'] !== null) {
              wpaltitude[index_rt][index]['wp'] = wp_count;
              wpaltitude[index_rt][index]['uavheight'] = (
                +wpaltitude[index_rt][index]['uav'] + +elevationprofile.results[initElevationIndex].elevation
              ).toFixed(1);
              wp_count = wp_count + 1;
            }
            auxcount = auxcount + 1;
          }
        }
      } catch (error) {
        console.log('Error in elevation profile:' + error);
        status = false;
      }
    }
    //console.log(wpaltitude);
    if (status) {
      return { elevation: wpaltitude, status: true };
    } else {
      return { elevation: [], status: false };
    }
  }

  static divideLineIntoPoints(line, steps, dist) {
    //console.log('line');
    //console.log(line);
    let dividedLine = []; // Start with the first point
    let accumulatedDistance = 0;
    let prevPoint = line[0];
    let currentPoint = line[1];
    let latStep = (currentPoint[0] - prevPoint[0]) / steps;
    let lngStep = (currentPoint[1] - prevPoint[1]) / steps;
    let distStep = dist / steps;
    for (let i = 1; i < steps; i++) {
      let newLat = prevPoint[0] + i * latStep;
      let newLon = prevPoint[1] + i * lngStep;
      let newdist = i * distStep;
      dividedLine.push({ lat: newLat, lon: newLon, dist: newdist });
    }

    //dividedLine.push(line[line.length - 1]); // Add the last point
    //console.log('dividedLine------------');
    //console.log(dividedLine);
    return dividedLine;
  }
}
