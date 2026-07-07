import { Fragment, useState, useMemo } from 'react';
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
import { useAsyncTask } from '../../reactHelper';

const useStyles = makeStyles()((theme) => ({
  chart: {
    position: 'relative',
    flexGrow: 1,
    overflow: 'hidden',
  },
  chartOverlay: {
    position: 'absolute',
    inset: 0,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    pointerEvents: 'none',
    color: 'rgba(0, 0, 0, 0.4)',
  },
  content: {
    height: '100%',
    flexGrow: 1,
    alignItems: 'stretch',
    display: 'flex',
    flexDirection: 'column',
    overflowY: 'auto',
  },
  tooltipContainer: {
    background: theme.palette.background.paper,
    padding: '8px',
    border: `1px solid ${theme.palette.divider}`,
    borderRadius: '4px',
    boxShadow: theme.shadows[2],
  },
  tooltipLabel: {
    marginBottom: '6px',
    fontWeight: 500,
  },
  tooltipRoute: {
    fontSize: '14px',
    marginTop: '4px',
  },
}));

const buildTooltipItems = (payload) => {
  const list = [{ ...payload[0].payload, color: payload[0].color }];
  payload.forEach((item) => {
    if (!list.some((existing) => item.payload.rt === existing.rt)) {
      list.push({ ...item.payload, color: item.color });
    }
  });
  return list;
};

const CustomTooltip = ({ active, payload, label }) => {
  const { classes } = useStyles();
  if (!active || !payload?.length) return null;

  const list = buildTooltipItems(payload);

  return (
    <div className={classes.tooltipContainer}>
      <div className={classes.tooltipLabel}>{`Distancia: ${label} m`}</div>
      {list.map((item) => (
        <Fragment key={`tooltip-${item.rt}`}>
          <div className={classes.tooltipRoute} style={{ color: item.color }}>
            {`Ruta ${item.rt} - WP ${item.wp}`}
          </div>
          <div className={classes.tooltipRoute} style={{ color: item.color }}>
            {item.uavheight
              ? `Terreno: ${item.elevation} m - UAV: ${item.uavheight} m`
              : `Terreno: ${item.elevation} m`}
          </div>
        </Fragment>
      ))}
    </div>
  );
};

const getWpListFromRoute = (auxroute) =>
  auxroute.map((route) => route.wp.map((wp) => [wp.pos[0], wp.pos[1], wp.pos[2]]));

const getCoordinatesFromProfile = (profile) =>
  profile.map((route) =>
    route.data
      .filter((point) => point.wp !== undefined)
      .map((point) => [point.lat, point.lng, point.uav])
  );

const hasGeometryChanged = (currentWaypoints, profileCoordinates) => {
  if (currentWaypoints.length !== profileCoordinates.length) return true;

  return currentWaypoints.some((currentRoute, i) => {
    const prevRoute = profileCoordinates[i] || [];
    if (currentRoute.length !== prevRoute.length) return true;

    return currentRoute.some(
      (wp, j) => wp[0] !== prevRoute[j]?.[0] || wp[1] !== prevRoute[j]?.[1]
    );
  });
};

const applyAltitudeDelta = (elevProfile, currentWaypoints, profileCoordinates) => {
  const updated = structuredClone(elevProfile);
  let changed = false;

  currentWaypoints.forEach((currentRoute, i) => {
    if (!updated[i]?.data) return;

    const prevRoute = profileCoordinates[i] || [];
    currentRoute.forEach((wp, j) => {
      const prevAltitude = prevRoute[j]?.[2];
      if (prevAltitude === undefined || wp[2] === prevAltitude) return;

      changed = true;
      updated[i].data.forEach((point) => {
        if (point.wp === j) {
          point.uavheight = +point.uavheight - +point.uav + wp[2];
          point.uav = wp[2];
        }
      });
    });
  });

  return changed ? updated : null;
};

const MissionElevation = ({ active = true }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const [selectRT, setSelectRT] = useState(-1);
  const [loading, setLoading] = useState(false);

  const missionRoute = useSelector((state) => state.mission.route);
  const elevProfile = useSelector((state) => state.mission.elevation.profile);

  const missionGeometry = useMemo(
    () => getWpListFromRoute(missionRoute),
    [missionRoute]
  );

  const selectedItems = useMemo(
    () => (selectRT === -1 ? elevProfile : elevProfile[selectRT] ? [elevProfile[selectRT]] : []),
    [selectRT, elevProfile]
  );

  const chartData = useMemo(() => selectedItems.flatMap((item) => item.data), [selectedItems]);

  const { minValue, maxValue, range } = useMemo(() => {
    const elevations = chartData.map((p) => Number(p.elevation));
    const uavHeights = chartData.map((p) => (p.uavheight ? Number(p.uavheight) : 0));

    const min = elevations.length > 0 ? Math.min(...elevations) : 0;
    const max = uavHeights.length > 0 ? Math.max(...uavHeights) : 0;
    const span = max - min || 10;

    return { minValue: min, maxValue: max, range: span };
  }, [chartData]);

  useAsyncTask(
    async ({ signal }) => {
      if (missionRoute.length === 0) return;

      const currentWaypoints = missionGeometry;
      const profileCoordinates = elevProfile.length > 0 ? getCoordinatesFromProfile(elevProfile) : [];

      const hasLastRouteWaypoints = currentWaypoints[currentWaypoints.length - 1]?.length > 0;
      if (!hasLastRouteWaypoints) return;

      if (!hasGeometryChanged(currentWaypoints, profileCoordinates)) {
        const updatedProfile = applyAltitudeDelta(elevProfile, currentWaypoints, profileCoordinates);
        if (updatedProfile) {
          dispatch(missionActions.setElevationProfile(updatedProfile));
        }
        return;
      }

      setLoading(true);
      try {
        const response = await fetch('/api/map/elevation', {
          method: 'POST',
          headers: { Accept: 'application/json', 'Content-Type': 'application/json' },
          body: JSON.stringify({ routes: currentWaypoints }),
          signal,
        });

        if (!response.ok) throw new Error(`HTTP ${response.status}`);

        const result = await response.json();
        if (result.status && result.elevation) {
          const elevationData = result.elevation.map((route, idx) => ({
            name: `RT${idx}`,
            data: route,
            color: palette.colors_devices[missionRoute[idx]?.id],
          }));
          setSelectRT(-1);
          dispatch(missionActions.setElevationProfile(elevationData));
        }
      } finally {
        setLoading(false);
      }
    },
    [missionGeometry]
  );

  return (
    <div className={classes.content}>
      <Box sx={{ display: 'flex', margin: '10px', alignItems: 'center' }}>
        <span style={{ marginInline: '20px' }}>Elevation Profile</span>
        <FormControl size="small">
          <InputLabel id="elevation-route-select-label">Route</InputLabel>
          <Select
            labelId="elevation-route-select-label"
            id="elevation-route-select"
            value={selectRT}
            label="Route"
            onChange={(e) => setSelectRT(e.target.value)}
          >
            <MenuItem value={-1}>All Routes</MenuItem>
            {elevProfile.map((_, index) => (
              <MenuItem key={`route-${index}`} value={index}>
                Route {index}
              </MenuItem>
            ))}
          </Select>
        </FormControl>
        {loading && <CircularProgress size={20} sx={{ ml: 2 }} />}
      </Box>

      {active && (
        <div className={classes.chart}>
          <ResponsiveContainer>
            <LineChart data={chartData} margin={{ top: 10, right: 40, left: 0, bottom: 10 }}>
              <XAxis dataKey="length" type="number" domain={['dataMin', 'dataMax']} />
              <YAxis
                type="number"
                tickFormatter={(value) => value.toFixed(2)}
                domain={[minValue - range / 5, maxValue + range / 5]}
              />
              <CartesianGrid strokeDasharray="3 3" />
              <Tooltip content={<CustomTooltip />} />
              <Legend />
              {selectedItems.map((item, idx) => (
                <Fragment key={`route-${idx}`}>
                  <Line
                    type="monotone"
                    dataKey="elevation"
                    data={item.data}
                    name={item.name}
                    stroke={item.color}
                    activeDot={{ r: 8 }}
                  />
                  <Line
                    connectNulls
                    dataKey="uavheight"
                    data={item.data}
                    name={`${item.name}-v`}
                    stroke={item.color}
                    strokeDasharray="5 5"
                  />
                </Fragment>
              ))}
            </LineChart>
          </ResponsiveContainer>
          {chartData.length === 0 && (
            <div className={classes.chartOverlay}>Sin datos de elevación</div>
          )}
        </div>
      )}
    </div>
  );
};

export default MissionElevation;
