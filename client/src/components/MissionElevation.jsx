import React, { useEffect, Fragment, useRef, useCallback } from 'react';
import { useSelector, useDispatch } from 'react-redux';
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
import { FormControl, InputLabel, Select, Box, MenuItem, CircularProgress } from '@mui/material';
import palette from '../common/palette';
import { makeStyles } from 'tss-react/mui';
import { missionActions } from '../store';

const useStyles = makeStyles()((theme) => ({
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
  loading: {
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    height: '100%',
  },
}));

const MissionElevation = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  // Get state from Redux
  const missionRoute = useSelector((state) => state.mission.route);
  const elevProfile = useSelector((state) => state.mission.elevation.profile);
  const location = useSelector((state) => state.mission.elevation.location);
  const selectRT = useSelector((state) => state.mission.elevation.selectRT);
  const loading = useSelector((state) => state.mission.elevation.loading);

  // Ref for accessing latest elevProfile in callbacks
  const elevProfileRef = useRef(elevProfile);
  elevProfileRef.current = elevProfile;

  // Computed items based on selectRT
  const items = selectRT === -1 ? elevProfile : (elevProfile[selectRT] ? [elevProfile[selectRT]] : []);

  // Chart value calculation
  const chartValue = React.useMemo(() => {
    const routes = items.map((it) => it.data);
    const values = routes.flat().map((it) => it['elevation']);
    const values1 = routes.flat().map((it) => (it['uavheight'] ? Number(it['uavheight']) : 0));
    const minValueAux = values.length > 0 ? Math.min(...values) : 0;
    const maxValueAux = values1.length > 0 ? Math.max(...values1) : 0;
    return { min: minValueAux, max: maxValueAux, range: maxValueAux - minValueAux || 10 };
  }, [items]);

  const getWpList = useCallback((auxroute) => {
    const listwp = [];
    if (auxroute.length > 0) {
      auxroute.forEach((route) => {
        const listwpAux = [];
        route.wp.forEach((wp) => {
          listwpAux.push([wp.pos[0], wp.pos[1], wp.pos[2]]);
        });
        listwp.push(listwpAux);
      });
    }
    return listwp;
  }, []);

  const fetchElevation = useCallback(async () => {
    console.log('--------------   LLAMANDO A ELEVATION');
    if (missionRoute.length === 0) return;

    dispatch(missionActions.setElevationLoading(true));

    const auxroute = JSON.parse(JSON.stringify(missionRoute));
    const ruteColor = auxroute.map((element) => element.id);
    const listwp = getWpList(auxroute);
    dispatch(missionActions.setElevationLocation(listwp));

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
      console.log('error to call server /api/map/elevation');
      console.log(error);
      dispatch(missionActions.setElevationLoading(false));
      return;
    }

    if (command.status) {
      const elevationRoute = command.elevation.map((route, index_rt) => ({
        name: 'RT' + index_rt,
        data: route,
        color: palette.colors_devices[ruteColor[index_rt]],
      }));
      dispatch(missionActions.setElevationSelectRT(-1));
      dispatch(missionActions.setElevationProfile(elevationRoute));
    } else {
      console.log('command', command);
      console.log('no get elevation of point error to connect to API of elevation');
    }
    dispatch(missionActions.setElevationLoading(false));
  }, [missionRoute, dispatch, getWpList]);

  // Handle route changes
  useEffect(() => {
    let change = false;
    const currentLocation = getWpList(missionRoute);
    console.log('update elevation');

    if (currentLocation.length > location.length) {
      // New route created
      console.log('ruta mayor');
      if (currentLocation[currentLocation.length - 1].length > 0) {
        change = true;
      } else {
        dispatch(missionActions.setElevationLocation(currentLocation));
      }
    } else if (currentLocation.length < location.length) {
      // Route deleted
      console.log('se ha eliminado una ruta');
      const indicesToKeep = [];
      location.forEach((key, index) => {
        const checkNodelete = currentLocation.some(
          (ckey) => JSON.stringify(key) === JSON.stringify(ckey)
        );
        if (checkNodelete) {
          indicesToKeep.push(index);
        }
      });
      dispatch(missionActions.removeElevationRoute(indicesToKeep));
      dispatch(missionActions.setElevationLocation(currentLocation));
    } else {
      // Same number of routes
      console.log('ruta de la misma longitud');
      for (let i = 0; i < currentLocation.length; i++) {
        if (JSON.stringify(currentLocation[i]) !== JSON.stringify(location[i])) {
          console.log('latitude longitude no equals');
          const latlonloc = location[i]?.map((wp) => [wp[0], wp[1]]) || [];
          const latloncur = currentLocation[i].map((wp) => [wp[0], wp[1]]);

          if (JSON.stringify(latlonloc) === JSON.stringify(latloncur)) {
            // Only altitude changed
            for (let j = 0; j < (location[i]?.length || 0); j++) {
              if (location[i][j][2] !== currentLocation[i][j][2]) {
                console.log('diferent elevation');
                const auxElevprofile = JSON.parse(JSON.stringify(elevProfileRef.current));
                if (auxElevprofile[i]?.data) {
                  for (let k = 0; k < auxElevprofile[i].data.length; k++) {
                    if (auxElevprofile[i].data[k].hasOwnProperty('wp')) {
                      if (auxElevprofile[i].data[k].wp === j) {
                        console.log('encontrado elev profile wp');
                        auxElevprofile[i].data[k].uavheight =
                          +auxElevprofile[i].data[k].uavheight -
                          +auxElevprofile[i].data[k].uav +
                          +currentLocation[i][j][2];
                        auxElevprofile[i].data[k].uav = +currentLocation[i][j][2];
                      }
                    }
                  }
                  dispatch(missionActions.setElevationProfile(auxElevprofile));
                }
              }
            }
            dispatch(missionActions.setElevationLocation(currentLocation));
            break;
          } else {
            // Lat/lon changed - need to recalculate
            console.log('nuevas posiciones');
            change = true;
            break;
          }
        }
      }
    }

    if (change) {
      fetchElevation();
    }
  }, [missionRoute, location, dispatch, getWpList, fetchElevation]);

  const handleSelectChange = (event) => {
    dispatch(missionActions.setElevationSelectRT(event.target.value));
  };

  const CustomTooltip = ({ active, payload, label }) => {
    if (active && payload && payload.length) {
      const list = [{ ...payload[0].payload, color: payload[0].color }];
      payload.forEach((key) => {
        const match = list.every((listkey) => key.payload.rt !== listkey.rt);
        if (match) {
          list.push({ ...key.payload, color: key.color });
        }
      });

      if (list[0].uavheight) {
        return (
          <div style={{ background: '#FFFFFF', padding: '5px', border: '1px solid #ccc' }}>
            <div>{`distancia ${label} m `}</div>
            {list.map((key) => (
              <Fragment key={'s-' + key.rt}>
                <div style={{ color: key.color, fontSize: 16 }}>
                  {`Ruta ${key.rt} - wp ${key.wp}`}
                </div>
                <div style={{ color: key.color, fontSize: 16 }}>
                  {`Terreno:${key.elevation} - UAV: ${key.uavheight}`}
                </div>
              </Fragment>
            ))}
          </div>
        );
      }
      return (
        <div style={{ background: '#FFFFFF', padding: '5px', border: '1px solid #ccc' }}>
          <div>{`distancia ${label} m `}</div>
          {list.map((key) => (
            <Fragment key={'s-' + key.rt}>
              <div style={{ color: key.color, fontSize: 16 }}>{`Ruta ${key.rt}`}</div>
              <div style={{ color: key.color, fontSize: 16 }}>{`Terreno:${key.elevation} `}</div>
            </Fragment>
          ))}
        </div>
      );
    }
    return null;
  };

  return (
    <div className={classes.content}>
      <Box style={{ display: 'flex', margin: '10px', alignItems: 'center' }}>
        <span style={{ marginInline: '20px' }}>Elevation Profile</span>
        <FormControl size="small">
          <InputLabel id="elevation-route-select-label">Route</InputLabel>
          <Select
            labelId="elevation-route-select-label"
            id="elevation-route-select"
            value={selectRT}
            label="Route"
            onChange={handleSelectChange}
          >
            <MenuItem value={-1}>All Routes</MenuItem>
            {elevProfile.map((_, index) => (
              <MenuItem key={`sel${index}`} value={index}>{`Route ${index}`}</MenuItem>
            ))}
          </Select>
        </FormControl>
        {loading && <CircularProgress size={20} sx={{ ml: 2 }} />}
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
                domain={[
                  chartValue.min - chartValue.range / 5,
                  chartValue.max + chartValue.range / 5,
                ]}
              />
              <CartesianGrid strokeDasharray="3 3" />
              <Tooltip content={<CustomTooltip />} />
              <Legend />
              {items.map((s, s_index) => (
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
                        const url = `https://www.google.com/maps?q=${payload.payload.lat},${payload.payload.lng}`;
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
