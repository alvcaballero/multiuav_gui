import { useEffect, Fragment, useState } from 'react';
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
import palette from '../../shared/palette';
import { makeStyles } from 'tss-react/mui';
import { missionActions } from '../../store';

const useStyles = makeStyles()(() => ({
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

const CustomTooltip = ({ active, payload, label }) => {
  if (!active || !payload?.length) return null;

  const list = [{ ...payload[0].payload, color: payload[0].color }];
  payload.forEach((key) => {
    if (list.every((listkey) => key.payload.rt !== listkey.rt)) {
      list.push({ ...key.payload, color: key.color });
    }
  });

  return (
    <div style={{ background: '#FFFFFF', padding: '5px', border: '1px solid #ccc' }}>
      <div>{`distancia ${label} m `}</div>
      {list.map((key) => (
        <Fragment key={'s-' + key.rt}>
          <div style={{ color: key.color, fontSize: 16 }}>{`Ruta ${key.rt} - wp ${key.wp}`}</div>
          <div style={{ color: key.color, fontSize: 16 }}>
            {key.uavheight
              ? `Terreno:${key.elevation} - UAV: ${key.uavheight}`
              : `Terreno:${key.elevation}`}
          </div>
        </Fragment>
      ))}
    </div>
  );
};

const getWpList = (auxroute) =>
  auxroute.map((route) => route.wp.map((wp) => [wp.pos[0], wp.pos[1], wp.pos[2]]));

const MissionElevation = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  // Defer chart mount so ResponsiveContainer measures real DOM dimensions (React19 compat)
  const [chartMounted, setChartMounted] = useState(false);
  useEffect(() => {
    setChartMounted(true);
    return () => setChartMounted(false);
  }, []);

  const missionRoute = useSelector((state) => state.mission.route);
  const elevProfile = useSelector((state) => state.mission.elevation.profile);
  const location = useSelector((state) => state.mission.elevation.location);
  const selectRT = useSelector((state) => state.mission.elevation.selectRT);
  const loading = useSelector((state) => state.mission.elevation.loading);

  const items =
    selectRT === -1 ? elevProfile : elevProfile[selectRT] ? [elevProfile[selectRT]] : [];

  const routes = items.map((it) => it.data);
  const elevValues = routes.flat().map((it) => it['elevation']);
  const uavValues = routes.flat().map((it) => (it['uavheight'] ? Number(it['uavheight']) : 0));
  const minValue = elevValues.length > 0 ? Math.min(...elevValues) : 0;
  const maxValue = uavValues.length > 0 ? Math.max(...uavValues) : 0;
  const range = maxValue - minValue || 10;

  useEffect(() => {
    const fetchElevation = async (listwp, ruteColor) => {
      dispatch(missionActions.setElevationLoading(true));
      dispatch(missionActions.setElevationLocation(listwp));

      try {
        const response = await fetch('/api/map/elevation', {
          method: 'POST',
          headers: { Accept: 'application/json', 'Content-Type': 'application/json' },
          body: JSON.stringify({ routes: listwp }),
        });

        if (!response.ok) throw new Error(response.status);

        const command = await response.json();
        if (command.status) {
          const elevationRoute = command.elevation.map((route, index_rt) => ({
            name: 'RT' + index_rt,
            data: route,
            color: palette.colors_devices[ruteColor[index_rt]],
          }));
          dispatch(missionActions.setElevationSelectRT(-1));
          dispatch(missionActions.setElevationProfile(elevationRoute));
        }
      } catch (error) {
        console.error('Error fetching elevation:', error);
      } finally {
        dispatch(missionActions.setElevationLoading(false));
      }
    };

    if (missionRoute.length === 0) return;

    const currentLocation = getWpList(missionRoute);

    if (currentLocation.length > location.length) {
      if (currentLocation[currentLocation.length - 1].length > 0) {
        fetchElevation(
          currentLocation,
          missionRoute.map((el) => el.id),
        );
      } else {
        dispatch(missionActions.setElevationLocation(currentLocation));
      }
      return;
    }

    if (currentLocation.length < location.length) {
      const indicesToKeep = location.reduce((acc, key, index) => {
        if (currentLocation.some((ckey) => JSON.stringify(key) === JSON.stringify(ckey))) {
          acc.push(index);
        }
        return acc;
      }, []);
      dispatch(missionActions.removeElevationRoute(indicesToKeep));
      dispatch(missionActions.setElevationLocation(currentLocation));
      return;
    }

    // Same number of routes — check for changes
    for (let i = 0; i < currentLocation.length; i++) {
      if (JSON.stringify(currentLocation[i]) === JSON.stringify(location[i])) continue;

      const latlonloc = location[i]?.map((wp) => [wp[0], wp[1]]) || [];
      const latloncur = currentLocation[i].map((wp) => [wp[0], wp[1]]);

      if (JSON.stringify(latlonloc) !== JSON.stringify(latloncur)) {
        fetchElevation(
          currentLocation,
          missionRoute.map((el) => el.id),
        );
        return;
      }

      // Only altitude changed — update profile in place without re-fetching
      const auxElevprofile = JSON.parse(JSON.stringify(elevProfile));
      for (let j = 0; j < (location[i]?.length || 0); j++) {
        if (location[i][j][2] === currentLocation[i][j][2]) continue;
        if (!auxElevprofile[i]?.data) continue;

        auxElevprofile[i].data.forEach((point) => {
          if (point.hasOwnProperty('wp') && point.wp === j) {
            point.uavheight = +point.uavheight - +point.uav + +currentLocation[i][j][2];
            point.uav = +currentLocation[i][j][2];
          }
        });
      }
      dispatch(missionActions.setElevationProfile(auxElevprofile));
      dispatch(missionActions.setElevationLocation(currentLocation));
      return;
    }
  }, [missionRoute, location, dispatch, elevProfile]);

  const handleSelectChange = (event) => {
    dispatch(missionActions.setElevationSelectRT(event.target.value));
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

      {chartMounted && items.length > 0 && (
        <div className={classes.chart}>
          <ResponsiveContainer>
            <LineChart data={items} margin={{ top: 10, right: 40, left: 0, bottom: 10 }}>
              <XAxis dataKey="length" type="number" domain={['dataMin', 'dataMax']} />
              <YAxis
                type="number"
                tickFormatter={(value) => value.toFixed(2)}
                domain={[minValue - range / 5, maxValue + range / 5]}
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
                    activeDot={{ r: 8 }}
                  />
                  <Line
                    connectNulls
                    dataKey="uavheight"
                    data={s.data}
                    name={s.name + '-v'}
                    key={s.name + '-v'}
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
