import React, { useState, useEffect, Fragment } from "react";
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
import { FormControl, InputLabel, Select, Box, MenuItem } from "@mui/material";
import { colors } from "../Mapview/preloadImages";
import { makeStyles } from "@mui/styles";
//https://www.opentopodata.org/
//https://open-elevation.com/
//https://codepen.io/tobinbradley/pen/jOeRbwr
// https://github.com/traccar/traccar-web/blob/667d0f68daaa3916fac6653ef9994f6f2d05e177/modern/src/reports/ChartReportPage.jsx#L8
const useStyles = makeStyles((theme) => ({
  chart: {
    flexGrow: 1,
    overflow: "hidden",
  },
  content: {
    height: "100%",
    flexGrow: 1,
    alignItems: "stretch",
    display: "flex",
    flexDirection: "column",
    overflowY: "auto",
  },
}));

const MissionElevation = () => {
  const classes = useStyles();
  const [items, setItems] = useState([]);
  const [ElevProfile, setElevProfile] = useState([]);
  const [SelectRT, setSelectRT] = useState(-1);
  const Mission_route = useSelector((state) => state.mission);

  const values = items.map((it) => it["elevation"]);
  const values1 = items.map((it) => it["uavheight"]);
  const minValue = Math.min(...values);
  const maxValue = Math.max(...values1);
  const valueRange = maxValue - minValue;

  const data = [
    {
      name: "RT0",
      data: [
        {
          length: 0,
          uavheight: 20.5,
          elevation: 15.5,
          wp: 0,
          rt: 0,
        },
        {
          length: 4,
          uavheight: 20.5,
          elevation: 20.5,
          wp: 0,
          rt: 0,
        },
        {
          length: 7.5,
          uavheight: 20.5,
          elevation: 1.5,
          wp: 1,
          rt: 0,
        },
        {
          length: 18.5,
          uavheight: 20.5,
          elevation: 15.5,
          wp: 2,
          rt: 0,
        },
        {
          length: 25.8,
          uavheight: 20.5,
          elevation: 15.5,
          wp: 3,
          rt: 0,
        },
      ],
      color: "#F34C28",
    },
    {
      name: "RT1",
      data: [
        {
          length: 0,
          uavheight: 10.5,
          elevation: 5.5,
          wp: 0,
          rt: 1,
        },
        {
          length: 4.8,
          uavheight: 10.5,
          elevation: 5.5,
          wp: 1,
          rt: 1,
        },
        {
          length: 10.2,
          uavheight: 10.5,
          elevation: 5.4,
          wp: 2,
          rt: 1,
        },
        {
          length: 17.1,
          uavheight: 10.5,
          elevation: 5.4,
          wp: 3,
          rt: 1,
        },
        {
          length: 24.4,
          uavheight: 10.5,
          elevation: 5.5,
          wp: 4,
          rt: 1,
        },
      ],
      color: "#F39A28",
    },
    {
      name: "RT2",
      data: [
        {
          length: 0,
          uavheight: 28.2,
          elevation: 6.2,
          wp: 0,
          rt: 2,
        },
        {
          length: 6.3,
          uavheight: 26.2,
          elevation: 8.6,
          wp: 1,
          rt: 2,
        },
        {
          length: 13.9,
          uavheight: 36.2,
          elevation: 8.2,
          wp: 2,
          rt: 2,
        },
        {
          length: 18.5,
          uavheight: 26.2,
          elevation: 8.1,
          wp: 3,
          rt: 2,
        },
        {
          length: 22.9,
          uavheight: 31.2,
          elevation: 7.1,
          wp: 4,
          rt: 2,
        },
      ],
      color: "#1EC910",
    },
  ];

  async function elevation() {
    // mas robusto y llamar cuando cambie la altura del drone
    if (false) {
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
          throw new Error(response.status);
        }
      } catch (error) {
        console.log(error);
      }
      let elevationRoute = command.map((route, index_rt) => {
        return { name: "RT" + index_rt, data: route, color: colors[index_rt] };
      });
      console.log(elevationRoute);
      setSelectRT(-1);
      setElevProfile(elevationRoute);
      setItems(elevationRoute);
    } else {
      setSelectRT(-1);
      setElevProfile(data);
      setItems(data);
    }
  }

  useEffect(() => {
    console.log("mission Elevation");
    //elevation();
  }, []);
  useEffect(() => {
    elevation();
  }, [Mission_route]);
  useEffect(() => {
    if (SelectRT == -1) {
      setItems(ElevProfile);
    } else {
      setItems([ElevProfile[SelectRT]]);
    }
  }, [SelectRT]);

  const CustomTooltip = ({ active, payload, label }) => {
    //console.log(payload);
    if (active && payload && payload.length) {
      let list = [payload[0].payload];
      list[0]["color"] = payload[0].color;
      payload.map((key) => {
        let match = true;
        list.map((listkey) => {
          if (key.payload.rt == listkey.rt) {
            match = false;
          }
        });
        key.payload["color"] = key.color;
        match ? list.push(key.payload) : null;
      });
      //console.log(list);
      return (
        <div style={{ background: "#FFFFFF" }}>
          {list.map((key) => (
            <Fragment>
              <div
                key={`trt${key.rt}`}
                style={{ color: key.color, fontSize: 16 }}
              >{`Ruta ${key.rt} - wp ${key.wp}`}</div>
              <div
                key={`drt${key.rt}`}
                style={{ color: key.color, fontSize: 16 }}
              >{`Terreno:${key.elevation} - UAV: ${key.uavheight}`}</div>
            </Fragment>
          ))}
        </div>
      );
    }

    return null;
  };

  return (
    <div className={classes.content}>
      <Box style={{ display: "flex", margin: "10px", alignItems: "center" }}>
        <a style={{ marginInline: "20px" }}>Elevation Profile</a>
        <FormControl>
          <InputLabel id="demo-simple-select-label">Route</InputLabel>
          <Select
            labelId="demo-simple-select-label"
            id="demo-simple-select"
            value={SelectRT}
            label="Elevation Profile"
            onChange={(event) => setSelectRT(event.target.value)}
          >
            <MenuItem value={-1}>All Routes</MenuItem>
            {ElevProfile.map((name, index, other) => (
              <MenuItem
                key={`sel${index}`}
                value={index}
              >{`Route${index}`}</MenuItem>
            ))}
          </Select>
        </FormControl>
      </Box>

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
                domain={["dataMin", "dataMax"]}
              />
              <YAxis
                type="number"
                tickFormatter={(value) => value.toFixed(2)}
                domain={[minValue - valueRange / 5, maxValue + valueRange / 5]}
              />
              <CartesianGrid strokeDasharray="3 3" />
              <Tooltip content={<CustomTooltip />} />
              <Legend />
              {items.map((s, s_index, s_array) => (
                <Fragment key={"s-" + s_index}>
                  <Line
                    type="monotone"
                    dataKey="elevation"
                    data={s.data}
                    name={s.name}
                    key={s.name}
                    stroke={s.color}
                    activeDot={{ r: 8 }}
                  />
                  <Line
                    connectNulls
                    type="monotone"
                    dataKey="uavheight"
                    data={s.data}
                    name={s.name + "-v"}
                    key={s.name + "-v"}
                    stroke={s.color}
                    strokeDasharray="5 5"
                  />
                </Fragment>
              ))}
            </LineChart>
          </ResponsiveContainer>
        </div>
      )}
    </div>
  );
};

export default MissionElevation;
