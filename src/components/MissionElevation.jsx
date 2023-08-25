import React, { useState, useEffect } from "react";
import { useSelector } from "react-redux";
import * as turf from "@turf/turf"; //import { length } from "@turf/length";
import {
  CartesianGrid,
  Line,
  Legend,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";

import { makeStyles } from "@mui/styles";
//https://www.opentopodata.org/
//https://open-elevation.com/
//https://codepen.io/tobinbradley/pen/jOeRbwr
// https://github.com/traccar/traccar-web/blob/667d0f68daaa3916fac6653ef9994f6f2d05e177/modern/src/reports/ChartReportPage.jsx#L8
const useStyles = makeStyles((theme) => ({
  chart: {
    flexGrow: 1,
    maxWidth: `calc(100vw - 560px)`,
    height: "100%",
  },
}));

const MissionElevation = () => {
  const classes = useStyles();
  const [items, setItems] = useState([]);
  const Mission_route = useSelector((state) => state.mission);

  async function setnewaction() {
    let auxroute = JSON.parse(JSON.stringify(Mission_route.route));
    let listwaypoint = "";
    let listwp = [];
    let wpaltitude = [];
    if (auxroute.length > 0) {
      auxroute.forEach((route, index_rt, array_rt) => {
        let listwp_aux = [];
        route.wp.forEach((wp, index, array) => {
          listwaypoint = listwaypoint + wp.pos[0] + "," + wp.pos[1] + "|";
          listwp_aux.push([wp.pos[0], wp.pos[1]]);
          wpaltitude.push(wp.pos[2])
        });
        listwp.push(listwp_aux);
        if (array_rt.length - 1 == index_rt) {
          console.log("delete last value");
          listwaypoint = listwaypoint.slice(0, -1);
        }
      });
      console.log("Elevation-----------");
      console.log(listwp);
      var linestring = turf.lineString(listwp[0]);
      let lineLength = turf.length(linestring, { units: "meters" });
      let divisionLength = lineLength / 20;
      let newLine = turf.lineChunk(linestring, 10, {
        units: "meters",
      }).features;
      let wp_new = newLine.map((feature, index_ft, list_ft) => {
        if (list_ft.length - 1 == index_ft) {
          console.log("delete last value");
          return (
            feature.geometry.coordinates[0][0] +
            "," +
            feature.geometry.coordinates[0][1] +
            "|" +
            feature.geometry.coordinates[1][0] +
            "," +
            feature.geometry.coordinates[1][1]
          );
        }
        return (
          feature.geometry.coordinates[0][0] +
          "," +
          feature.geometry.coordinates[0][1] +
          "|"
        );
      });

      console.log(lineLength);
      console.log(divisionLength);
      console.log(linestring);
      console.log(newLine);
      console.log(wp_new);
    }

    let command;
    if (true) {
      try {
        const response = await fetch(
          `/api/map/elevation?locations=${listwaypoint}`
        );
        if (response.ok) {
          command = await response.json();
        } else {
          console.log("no ok response");
          console.log(response);
          throw new Error(response.status);
        }
      } catch (error) {
        console.log("error");
        console.log(error);
      }
    }
    console.log("data elevation");
    console.log(command.results);
    console.log(wpaltitude);
    //obener array de distancia
    let acumulative=[]
    let mydata = command.results.map((element, index, array) => {
      console.log(index);
      let lineLength = 0;
      acumulative.push(Object.values(element.location))
      if (index != 0) {
        let linestring = turf.lineString(acumulative);
        lineLength = turf.length(linestring, { units: "meters" });
      } 
      return { name: lineLength, cost: element.elevation, impression: command.results[0]["elevation"]+wpaltitude[index] };
    });
    console.log(mydata);
    setItems(mydata);
  }

  async function elevation() {
    console.log("elevation funtion");
  }

  const data = [
    { name: 1, cost: 4.11, impression: 300 },
    { name: 2, cost: 2.39, impression: 120 },
    { name: 3, cost: 1.37, impression: 150 },
    { name: 4, cost: 1.16, impression: 180 },
    { name: 5, cost: 2.29, impression: 200 },
    { name: 6, cost: 3 },
    { name: 7, cost: 0.53, impression: 50 },
    { name: 8, cost: 2.52, impression: 100 },
    { name: 9, cost: 1.79, impression: 200 },
    { name: 10, cost: 2.94, impression: 222 },
    { name: 11, cost: 4.3, impression: 210 },
    { name: 12, cost: 4.41, impression: 300 },
    { name: 13, cost: 2.1, impression: 50 },
    { name: 14, cost: 8, impression: 190 },
    { name: 15, cost: 0, impression: 300 },
    { name: 16, cost: 9, impression: 400 },
    { name: 17, cost: 3, impression: 200 },
    { name: 18, cost: 2 },
    { name: 19, cost: 3 },
    { name: 20, cost: 7 },
  ];
  useEffect(() => {
    console.log("mission Elevation");
    setItems(data);
  }, []);

  useEffect(() => {
    setnewaction();
  }, [Mission_route]);

  return (
    <div style={{ height: "100%", width: "100%" }}>
      {data.length > 0 && (
        <div className={classes.chart}>
          <ResponsiveContainer width="100%" height="100%">
            <LineChart
              data={items}
              margin={{
                top: 10,
                right: 40,
                left: 0,
                bottom: 10,
              }}
            >
              <XAxis
                dataKey="name"
                type="number"
                //tickFormatter={(value) => formatTime(value, "time", hours12)}
                //domain={["dataMin", "dataMax"]}
                //scale="time"
              />
              <YAxis
                type="number"
                tickFormatter={(value) => value.toFixed(2)}
                //domain={[minValue - valueRange / 5, maxValue + valueRange / 5]}
              />
              <CartesianGrid strokeDasharray="3 3" />
              <Tooltip />
              <Legend />
              <Line
                type="monotone"
                dataKey="cost"
                stroke="#8884d8"
                activeDot={{ r: 8 }}
              />
              <Line
                connectNulls
                type="monotone"
                dataKey="impression"
                stroke="#82ca9d"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      )}
    </div>
  );
};

export default MissionElevation;
