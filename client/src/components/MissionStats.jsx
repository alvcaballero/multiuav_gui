import React, { useMemo } from 'react';
import { useSelector } from 'react-redux';
import {
  Box,
  Paper,
  Typography,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Divider,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import RouteIcon from '@mui/icons-material/Route';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
import StraightenIcon from '@mui/icons-material/Straighten';
import PlaceIcon from '@mui/icons-material/Place';
import palette from '../common/palette';

const useStyles = makeStyles()((theme) => ({
  container: {
    padding: theme.spacing(2),
    height: '100%',
    overflowY: 'auto',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
    marginBottom: theme.spacing(2),
  },
  summaryCard: {
    padding: theme.spacing(2),
    marginBottom: theme.spacing(2),
    backgroundColor: theme.palette.grey[100],
  },
  summaryGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fit, minmax(120px, 1fr))',
    gap: theme.spacing(2),
  },
  statItem: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    textAlign: 'center',
  },
  statValue: {
    fontSize: '1.5rem',
    fontWeight: 'bold',
    color: theme.palette.primary.main,
  },
  statLabel: {
    fontSize: '0.75rem',
    color: theme.palette.text.secondary,
  },
  tableContainer: {
    maxHeight: 300,
  },
  routeColorCell: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
  },
  colorDot: {
    width: 12,
    height: 12,
    borderRadius: '50%',
  },
  noData: {
    textAlign: 'center',
    padding: theme.spacing(4),
    color: theme.palette.text.secondary,
  },
}));

/**
 * Calcula la distancia 2D entre dos puntos usando la fórmula Haversine
 * (Solo distancia horizontal, igual que turf.js en el servidor)
 * @param {number} lat1 - Latitud del punto 1
 * @param {number} lon1 - Longitud del punto 1
 * @param {number} lat2 - Latitud del punto 2
 * @param {number} lon2 - Longitud del punto 2
 * @returns {number} Distancia en metros
 *
 *
 */

const radiansToLength = (radians, units) => {
  //const earthRadius = 6371008.8;
  const factor = {
    kilometers: 6371,
    miles: 3958.8,
    meters: 6371000,
    feet: 20903520,
    degrees: 57.2958,
    radians: 1,
  }[units || 'meters'];
  return radians * factor;
};

const haversineDistance2D = (lat1, lon1, lat2, lon2) => {
  const toRad = (deg) => (deg * Math.PI) / 180;

  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);
  const lat1Rad = toRad(lat1);
  const lat2Rad = toRad(lat2);

  const a = Math.pow(Math.sin(dLat / 2), 2) + Math.pow(Math.sin(dLon / 2), 2) * Math.cos(lat1Rad) * Math.cos(lat2Rad);

  return radiansToLength(2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a)), 'meters');
};

/**
 * Calcula la distancia 3D entre dos puntos (incluye diferencia de altitud)
 * @param {number} lat1 - Latitud del punto 1
 * @param {number} lon1 - Longitud del punto 1
 * @param {number} alt1 - Altitud del punto 1
 * @param {number} lat2 - Latitud del punto 2
 * @param {number} lon2 - Longitud del punto 2
 * @param {number} alt2 - Altitud del punto 2
 * @returns {number} Distancia en metros
 */
const haversineDistance3D = (lat1, lon1, alt1, lat2, lon2, alt2) => {
  const horizontalDistance = haversineDistance2D(lat1, lon1, lat2, lon2);
  const verticalDistance = alt2 - alt1;
  return Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);
};

/**
 * Calcula la distancia total de una ruta
 * @param {Array} waypoints - Array de waypoints con pos [lat, lon, alt]
 * @param {boolean} include3D - Si true, calcula distancia 3D incluyendo altitud
 * @returns {number} Distancia total en metros
 */
const calculateRouteDistance = (waypoints, include3D = false) => {
  if (!waypoints || waypoints.length < 2) return 0;

  let totalDistance = 0;
  for (let i = 1; i < waypoints.length; i++) {
    const [lat1, lon1, alt1] = waypoints[i - 1].pos;
    const [lat2, lon2, alt2] = waypoints[i].pos;
    if (include3D) {
      if (i === 1) totalDistance += alt1;
      if (i === waypoints.length - 1) totalDistance += alt2;
      totalDistance += haversineDistance3D(lat1, lon1, alt1, lat2, lon2, alt2);
    } else {
      totalDistance += haversineDistance2D(lat1, lon1, lat2, lon2);
    }
  }
  return totalDistance;
};

/**
 * Estima el tiempo de vuelo basado en distancia y velocidad
 * @param {number} distance - Distancia en metros
 * @param {number} maxVel - Velocidad máxima en m/s
 * @param {number} idleVel - Velocidad idle en m/s
 * @returns {number} Tiempo estimado en segundos
 */
const estimateFlightTime = (distance, maxVel = 10, idleVel = 3) => {
  // Usar velocidad promedio entre max e idlek
  const avgVel = idleVel;
  return avgVel > 0 ? distance / avgVel : 0;
};

/**
 * Formatea segundos a formato legible (HH:MM:SS o MM:SS)
 * @param {number} seconds - Tiempo en segundos
 * @returns {string} Tiempo formateado
 */
const formatTime = (seconds) => {
  if (seconds < 60) {
    return `${Math.round(seconds)}s`;
  }
  const hrs = Math.floor(seconds / 3600);
  const mins = Math.floor((seconds % 3600) / 60);
  const secs = Math.round(seconds % 60);

  if (hrs > 0) {
    return `${hrs}h ${mins}m ${secs}s`;
  }
  return `${mins}m ${secs}s`;
};

/**
 * Formatea distancia a formato legible
 * @param {number} meters - Distancia en metros
 * @returns {string} Distancia formateada
 */
const formatDistance = (meters) => {
  if (meters >= 1000) {
    return `${(meters / 1000).toFixed(2)} km`;
  }
  return `${Math.round(meters)} m`;
};

const MissionStats = () => {
  const { classes } = useStyles();
  const missionRoutes = useSelector((state) => state.mission.route);

  const stats = useMemo(() => {
    if (!missionRoutes || missionRoutes.length === 0) {
      return null;
    }

    const routeStats = missionRoutes.map((route, index) => {
      const distance2D = calculateRouteDistance(route.wp, false);
      const distance3D = calculateRouteDistance(route.wp, true);
      const waypoints = route.wp?.length || 0;
      const maxVel = route.attributes?.max_vel || 10;
      const idleVel = route.attributes?.idle_vel || 3;
      // Use 3D distance for time estimation (more realistic)
      const time = estimateFlightTime(distance3D, maxVel, idleVel);

      return {
        id: route.id ?? index,
        name: route.name || `Route ${index}`,
        uav: route.uav || '-',
        distance2D,
        distance3D,
        waypoints,
        time,
        maxVel,
        idleVel,
      };
    });

    const totalStats = routeStats.reduce(
      (acc, route) => ({
        totalDistance2D: acc.totalDistance2D + route.distance2D,
        totalDistance3D: acc.totalDistance3D + route.distance3D,
        totalWaypoints: acc.totalWaypoints + route.waypoints,
        totalTime: acc.totalTime + route.time,
      }),
      { totalDistance2D: 0, totalDistance3D: 0, totalWaypoints: 0, totalTime: 0 }
    );

    return {
      totalRoutes: missionRoutes.length,
      ...totalStats,
      routeStats,
    };
  }, [missionRoutes]);

  if (!stats) {
    return (
      <Box className={classes.container}>
        <Box className={classes.header}>
          <RouteIcon />
          <Typography variant="h6">Mission Statistics</Typography>
        </Box>
        <Typography className={classes.noData}>No routes defined. Add waypoints to see statistics.</Typography>
      </Box>
    );
  }

  return (
    <Box className={classes.container}>
      <Box className={classes.header}>
        <RouteIcon />
        <Typography variant="h6">Mission Statistics</Typography>
      </Box>

      {/* Summary Card */}
      <Paper className={classes.summaryCard} elevation={0}>
        <Typography variant="subtitle2" gutterBottom>
          Total Summary
        </Typography>
        <Box className={classes.summaryGrid}>
          <Box className={classes.statItem}>
            <RouteIcon color="primary" />
            <Typography className={classes.statValue}>{stats.totalRoutes}</Typography>
            <Typography className={classes.statLabel}>Routes</Typography>
          </Box>
          <Box className={classes.statItem}>
            <PlaceIcon color="primary" />
            <Typography className={classes.statValue}>{stats.totalWaypoints}</Typography>
            <Typography className={classes.statLabel}>Waypoints</Typography>
          </Box>
          <Box className={classes.statItem}>
            <StraightenIcon color="primary" />
            <Typography className={classes.statValue}>{formatDistance(stats.totalDistance2D)}</Typography>
            <Typography className={classes.statLabel}>Distance 2D</Typography>
          </Box>
          <Box className={classes.statItem}>
            <StraightenIcon color="secondary" />
            <Typography className={classes.statValue}>{formatDistance(stats.totalDistance3D)}</Typography>
            <Typography className={classes.statLabel}>Distance 3D</Typography>
          </Box>
          <Box className={classes.statItem}>
            <AccessTimeIcon color="primary" />
            <Typography className={classes.statValue}>{formatTime(stats.totalTime)}</Typography>
            <Typography className={classes.statLabel}>Est. Time</Typography>
          </Box>
        </Box>
      </Paper>

      <Divider sx={{ my: 2 }} />

      {/* Routes Detail Table */}
      <Typography variant="subtitle2" gutterBottom>
        Routes Detail
      </Typography>
      <TableContainer component={Paper} className={classes.tableContainer}>
        <Table size="small" stickyHeader>
          <TableHead>
            <TableRow>
              <TableCell>Route</TableCell>
              <TableCell>UAV</TableCell>
              <TableCell align="right">WPs</TableCell>
              <TableCell align="right">Dist 2D</TableCell>
              <TableCell align="right">Dist 3D</TableCell>
              <TableCell align="right">Time</TableCell>
              <TableCell align="right">Vel (m/s)</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {stats.routeStats.map((route) => (
              <TableRow key={route.id} hover>
                <TableCell>
                  <Box className={classes.routeColorCell}>
                    <Box
                      className={classes.colorDot}
                      sx={{ backgroundColor: palette.colors_devices[route.id] || '#808080' }}
                    />
                    {route.name}
                  </Box>
                </TableCell>
                <TableCell>{route.uav}</TableCell>
                <TableCell align="right">{route.waypoints}</TableCell>
                <TableCell align="right">{formatDistance(route.distance2D)}</TableCell>
                <TableCell align="right">{formatDistance(route.distance3D)}</TableCell>
                <TableCell align="right">{formatTime(route.time)}</TableCell>
                <TableCell align="right">
                  {route.idleVel}-{route.maxVel}
                </TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </TableContainer>
    </Box>
  );
};

export default MissionStats;
