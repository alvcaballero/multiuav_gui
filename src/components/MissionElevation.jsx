import React, { useState, useEffect } from "react";
import { useSelector } from "react-redux";
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




   async function setnewaction (){
    console.log("remove action" + index_route + "-" + index_wp + "-" + action);
    let auxroute = JSON.parse(JSON.stringify(Mission_route.route));
    let listwaypoint = "";
    if (auxroute.length > 0) {
      auxroute.array.forEach((route) => {
        route.wp.forEach((wp) => {
            listwaypoint = listwaypoint + wp.pos[1], wp.pos[0]];
        });
      });
      console.log(listwaypoint);
    }

    
    let command;
    if (true) {
      const response = await fetch("/api/mission/actions/dji");
      if (response.ok) {
        command = await response.json();
      } else {
        throw Error(await response.text());
      }
    }
    console.log(command);

  };
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

  return (
    <div style={{ height: "100%", width: "100%" }}>
      {data.length > 0 && (
        <div className={classes.chart}>
          <ResponsiveContainer width="100%" height="100%">
            <LineChart
              data={data}
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
