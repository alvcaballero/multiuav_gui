import React, { useState, useEffect, Fragment, createRef } from 'react';
import { useSelector } from 'react-redux';
import {
  CartesianGrid,
  Line,
  Legend,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts';
import { FormControl, InputLabel, Select, Box, MenuItem } from '@mui/material';
import palette from '../common/palette';
import { makeStyles } from '@mui/styles';
//https://www.opentopodata.org/
//https://open-elevation.com/
//https://codepen.io/tobinbradley/pen/jOeRbwr
// https://github.com/traccar/traccar-web/blob/667d0f68daaa3916fac6653ef9994f6f2d05e177/modern/src/reports/ChartReportPage.jsx#L8
//https://medium.com/@onthegomap/new-and-improved-elevation-profiles-cb888b9ac8c4
const useStyles = makeStyles((theme) => ({
  chart: {
    flexGrow: 1,
    overflow: 'hidden',
  },
  content: {
    height: '100%',
    flexGrow: 1,
    alignItems: 'stretch',
    display: 'flex',
    flexDirection: 'column',
    overflowY: 'auto',
  },
}));

const MissionElevation = () => {
  const classes = useStyles();
  const [items, setItems] = useState([]);
  const [location, setlocation] = useState([]);
  const [ElevProfile, setElevProfile] = useState([]);
  const ElevProfileRef = React.createRef();
  ElevProfileRef.current = ElevProfile;
  const [SelectRT, setSelectRT] = useState(-1);
  const Mission_route = useSelector((state) => state.mission.route); //cambiar esto por  state.mission.route
  const [minValue, setMinValue] = useState(0);
  const [maxValue, setMaxValue] = useState(0);
  const [valueRange, setValueRange] = useState(0);

  useEffect(() => {
    console.log(items);
    const routes = items.map((it) => it.data);
    console.log(routes.flat());
    const values = routes.flat().map((it) => it['elevation']);
    const values1 = routes.flat().map((it) => (it['uavheight'] ? Number(it['uavheight']) : 0));
    console.log(values);
    console.log(values1);
    const minValueAux = Math.min(...values);
    const maxValueAux = Math.max(...values1);
    console.log('value' + minValueAux + 'max' + maxValueAux);
    setMinValue(minValueAux);
    setMaxValue(maxValueAux);
    setValueRange(maxValueAux - minValueAux);
  }, [items]);

  const GetWplist = (auxroute) => {
    let listwp = [];
    if (auxroute.length > 0) {
      auxroute.forEach((route) => {
        let listwp_aux = [];
        route.wp.forEach((wp) => {
          listwp_aux.push([wp.pos[0], wp.pos[1], wp.pos[2]]);
        });
        listwp.push(listwp_aux);
      });
    }
    return listwp;
  };

  async function elevation() {
    // mas robusto y llamar cuando cambie la altura del drone
    console.log('--------------   LLAMANDO A ELEVATION');
    if (Mission_route.length > 0) {
      let auxroute = JSON.parse(JSON.stringify(Mission_route));
      let ruteColor = auxroute.map((element) => element.id);
      let listwp = GetWplist(auxroute);
      setlocation(listwp);
      //console.log(listwp);

      let command;
      try {
        const response = await fetch('/api/map/elevation', {
          method: 'POST',
          headers: {
            Accept: 'application/json',
            'Content-Type': 'application/json',
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
      if (command.status) {
        let elevationRoute = command.elevation.map((route, index_rt) => {
          return {
            name: 'RT' + index_rt,
            data: route,
            color: palette.colors_devices[ruteColor[index_rt]],
          };
        });
        //console.log(elevationRoute);
        setSelectRT(-1);
        setElevProfile(elevationRoute);
      } else {
        console.log('no get elevation of point');
      }
    }
  }

  useEffect(() => {
    console.log('mission Elevation');
    //elevation();
  }, []);
  useEffect(() => {
    // el el caso de que haya cambiado la altura del uav  suamarla
    let change = false;
    let curentlocation = GetWplist(Mission_route);
    console.log('update elevation');
    //console.log(curentlocation);
    //console.log(location);
    if (curentlocation.length > location.length) {
      // se ha creado una ruta
      console.log('ruta mayour');
      if (curentlocation[+curentlocation.length + -1].length > 0) {
        change = true;
      } else {
        setlocation(curentlocation);
      }
    } else {
      if (curentlocation.length < location.length) {
        console.log(' se ha eliminado  una ruta');
        let auxlocation = [];
        let auxElevprofile = [];
        location.map((key, index) => {
          let check_nodelete = false;
          curentlocation.map((ckey) => {
            if (JSON.stringify(key) === JSON.stringify(ckey)) {
              check_nodelete = true;
            }
          });
          if (check_nodelete) {
            auxlocation.push(key);
            auxElevprofile.push(ElevProfileRef.current[index]);
          }
        });
        setElevProfile(auxElevprofile);
        setlocation(auxlocation);
      } else {
        console.log('ruta de la misma longitud');
        // ver si hay cambios de latitud y longitud
        for (let i = 0; i < curentlocation.length; i = i + 1) {
          //find changes in routes
          //console.log(JSON.stringify(curentlocation[i]));
          //console.log(JSON.stringify(location[i]));
          if (JSON.stringify(curentlocation[i]) != JSON.stringify(location[i])) {
            console.log('latitude longitude no equals');
            let latlonloc = location[i].map((wp) => [wp[0], wp[1]]);
            let latloncur = curentlocation[i].map((wp) => [wp[0], wp[1]]);
            if (JSON.stringify(latlonloc) === JSON.stringify(latloncur)) {
              //if latitude and longitude  are equal then change heigh
              for (let j = 0; j < location[i].length; j = j + 1) {
                if (location[i][j][2] !== curentlocation[i][j][2]) {
                  console.log('diferent elevation');
                  //cambiar la altura de este punto en la elevacion
                  let auxElevprofile = JSON.parse(JSON.stringify(ElevProfileRef.current));
                  for (let k = 0; k < auxElevprofile[i].data.length; k = k + 1) {
                    if (auxElevprofile[i]['data'][k].hasOwnProperty('wp')) {
                      if (auxElevprofile[i].data[k].wp == j) {
                        console.log('encontrado elev profile wp');
                        auxElevprofile[i].data[k].uavheight =
                          +auxElevprofile[i].data[k].uavheight -
                          +auxElevprofile[i].data[k].uav +
                          +curentlocation[i][j][2];
                        auxElevprofile[i].data[k].uav = +curentlocation[i][j][2];
                      }
                    }
                  }
                  console.log(auxElevprofile);
                  setElevProfile(auxElevprofile);
                }
              }
              setlocation(curentlocation);
              break;
            } else {
              //delete a waypoint -- a que las distancias entre los waypoints que  quedan son diferentes y tiene que se calculadas de nuevo
              //add  a waypoint
              console.log('nuevas posiciones');
              change = true;
              break;
            }
          }
        }
      }
    }
    //ejecutar elevacion solo si ha habido cambios de de latitud y longitud y creacion de puntos
    change ? elevation() : null;
  }, [Mission_route]);
  useEffect(() => {
    if (SelectRT == -1) {
      setItems(ElevProfile);
    } else {
      setItems([ElevProfile[SelectRT]]);
    }
  }, [SelectRT, ElevProfile]);

  const CustomTooltip = ({ active, payload, label }) => {
    if (active && payload && payload.length) {
      let list = [payload[0].payload];
      list[0]['color'] = payload[0].color;
      payload.map((key) => {
        let match = true;
        list.map((listkey) => {
          if (key.payload.rt == listkey.rt) {
            match = false;
          }
        });
        key.payload['color'] = key.color;
        match ? list.push(key.payload) : null;
      });
      if (list[0].uavheight) {
        return (
          <div style={{ background: '#FFFFFF' }}>
            <div>{`distancia ${label} m `}</div>
            {list.map((key) => (
              <Fragment key={'s-' + key.rt}>
                <div
                  key={`tr${key.rt}`}
                  style={{ color: key.color, fontSize: 16 }}
                >{`Ruta ${key.rt} - wp ${key.wp}`}</div>
                <div
                  key={`dr${key.rt}`}
                  style={{ color: key.color, fontSize: 16 }}
                >{`Terreno:${key.elevation} - UAV: ${key.uavheight}`}</div>
              </Fragment>
            ))}
          </div>
        );
      } else {
        return (
          <div style={{ background: '#FFFFFF' }}>
            <div>{`distancia ${label} m `}</div>
            {list.map((key) => (
              <Fragment key={'s-' + key.rt}>
                <div
                  key={`tr${key.rt}`}
                  style={{ color: key.color, fontSize: 16 }}
                >{`Ruta ${key.rt}`}</div>
                <div
                  key={`dr${key.rt}`}
                  style={{ color: key.color, fontSize: 16 }}
                >{`Terreno:${key.elevation} `}</div>
              </Fragment>
            ))}
          </div>
        );
      }
    }

    return null;
  };

  return (
    <div className={classes.content}>
      <Box style={{ display: 'flex', margin: '10px', alignItems: 'center' }}>
        <a style={{ marginInline: '20px' }}>Elevation Profile</a>
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
              <MenuItem key={`sel${index}`} value={index}>{`Route${index}`}</MenuItem>
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
              <XAxis dataKey="length" type="number" domain={['dataMin', 'dataMax']} />
              <YAxis
                type="number"
                tickFormatter={(value) => value.toFixed(2)}
                domain={[minValue - valueRange / 5, maxValue + valueRange / 5]}
              />
              <CartesianGrid strokeDasharray="3 3" />
              <Tooltip content={<CustomTooltip />} />
              <Legend />
              {items.map((s, s_index, s_array) => (
                <Fragment key={'s-' + s_index}>
                  <Line
                    type="monotone"
                    dataKey="elevation"
                    data={s.data}
                    name={s.name}
                    key={s.name}
                    stroke={s.color}
                    activeDot={{
                      r: 8,
                      onClick: (event, payload) => {
                        console.log(payload);
                        let url = `https://www.google.com/maps?q=${payload.payload.lat},${payload.payload.lng}`;
                        window.open(url, '_blank');
                      },
                    }}
                  />
                  <Line
                    connectNulls
                    dataKey="uavheight"
                    data={s.data}
                    name={s.name + '-v'}
                    key={s.name + '-v'}
                    stroke={s.color}
                    strokeDasharray="5 5"
                    activeDot={{
                      onClick: (event, payload) => {
                        console.log(payload);
                      },
                    }}
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
