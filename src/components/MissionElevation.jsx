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
import { colors } from "../Mapview/preloadImages";
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
          wpaltitude.push(wp.pos[2]);
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
    let acumulative = [];
    let mydata = command.results.map((element, index, array) => {
      console.log(index);
      let lineLength = 0;
      acumulative.push(Object.values(element.location));
      if (index != 0) {
        let linestring = turf.lineString(acumulative);
        lineLength = turf.length(linestring, { units: "meters" });
      }
      return {
        name: lineLength,
        cost: element.elevation,
        impression: command.results[0]["elevation"] + wpaltitude[index],
      };
    });
    console.log(mydata);
    setItems(mydata);
  }

  async function elevation() {
    console.log("elevation funtion");
    let auxroute = JSON.parse(JSON.stringify(Mission_route.route));
    let listwp = [];
    if (auxroute.length > 0) {
      auxroute.forEach((route, index_rt, array_rt) => {
        let listwp_aux = [];
        route.wp.forEach((wp, index, array) => {
          listwp_aux.push([wp.pos[0], wp.pos[1], wp.pos[2]]);
        });
        listwp.push(listwp_aux);
      });
      console.log("Elevation-----------");
      console.log(listwp);
    }
    let command;
    try {
      const response = await fetch("/api/map/elevation", {
        method: "POST",
        headers: {
          Accept: "application/json",
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ routes: listwp }),
      });
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
    let elevationRoute = command.map((route, index_rt) => {
      return { name: "Route" + index_rt, data: route };
    });
    console.log(elevationRoute);
    setItems(elevationRoute);
  }

  useEffect(() => {
    console.log("mission Elevation");
    //setItems(data);
  }, []);

  useEffect(() => {
    elevation();
  }, [Mission_route]);

  return (
    <div style={{ height: "100%", width: "100%" }}>
      {items.length > 0 && (
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
                dataKey="length"
                type="number"
                //tickFormatter={(value) => formatTime(value, "time", hours12)}
                //domain={["dataMin", "dataMax"]}
                scale="meters"
              />
              <YAxis
                type="number"
                tickFormatter={(value) => value.toFixed(2)}
                //domain={[minValue - valueRange / 5, maxValue + valueRange / 5]}
              />
              <CartesianGrid strokeDasharray="3 3" />
              <Tooltip />
              <Legend />
              {items.map((s, s_index, s_array) => (
                <>
                  <Line
                    type="monotone"
                    dataKey="elevation"
                    data={s.data}
                    name={s.name}
                    key={s.name}
                    stroke={colors[s_index]}
                    strokeDasharray="5 5"
                    activeDot={{ r: 8 }}
                  />
                  <Line
                    connectNulls
                    type="monotone"
                    dataKey="uavheight"
                    data={s.data}
                    name={s.name + "uav"}
                    key={s.name + "uav"}
                    stroke={colors[s_index]}
                  />
                </>
              ))}
            </LineChart>
          </ResponsiveContainer>
        </div>
      )}
    </div>
  );
};

export default MissionElevation;
