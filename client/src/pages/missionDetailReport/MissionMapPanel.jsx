import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useSelector } from 'react-redux';
import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Box,
  Divider,
  Paper,
  Slider,
  Tab,
  Typography,
} from '@mui/material';
import { TabContext, TabList, TabPanel } from '@mui/lab';
import ExpandMore from '@mui/icons-material/ExpandMore';

import { formatTime } from '../../shared/formatter';
import { map } from '../../map/core/mapInstance';
import MapView from '../../map/core/MapView';
import { MapMissions } from '../../map/mission/MapMissions';
import MapFlightPath from '../../map/mission/MapFlightPath';
import MapMarkers from '../../map/environment/MapMarkers';
import MapPositions from '../../map/devices/MapPositions';
import RoutesList from '../../components/mission/RoutesList';
import SelectField from '../../shared/components/SelectField';
import SelectList from '../../components/ui/SelectList';
import BaseSettings from '../../components/planning/BaseSettings';

const noop = () => null;
const EMPTY_TRACKS = [];

const fitBoundsToRoute = (routePath, flightTracks) => {
  const waypointCoords = (routePath?.flatMap((route) => route.wp ?? []) ?? []).map((wp) => [
    wp.pos[1],
    wp.pos[0],
  ]);
  const trackCoords = flightTracks?.flatMap((track) => track.points) ?? [];
  const coords = [...waypointCoords, ...trackCoords];
  if (coords.length === 0) return;

  const lons = coords.map(([lon]) => lon);
  const lats = coords.map(([, lat]) => lat);
  map.fitBounds(
    [
      [Math.min(...lons), Math.min(...lats)],
      [Math.max(...lons), Math.max(...lats)],
    ],
    { padding: 60, animate: false },
  );
};

// Positions come sorted by fixTime; pick the recorded fix closest to `time`.
const nearestPosition = (positions, time) =>
  positions.reduce((closest, position) => {
    if (!closest) return position;
    const closestDiff = Math.abs(new Date(closest.fixTime).getTime() - time);
    const diff = Math.abs(new Date(position.fixTime).getTime() - time);
    return diff < closestDiff ? position : closest;
  }, null);

const MissionMapPanel = ({
  classes,
  missions,
  routePath,
  missionMarkers,
  routeTracks = EMPTY_TRACKS,
  dataMission,
  dataParam,
  tabValue,
  onTabChange,
}) => {
  const devices = useSelector((state) => state.devices.items);

  const flightTracks = useMemo(
    () =>
      routeTracks.map((track) => ({
        id: track.id,
        deviceId: track.deviceId,
        points: track.positions.map((position) => [position.longitude, position.latitude]),
      })),
    [routeTracks],
  );

  useEffect(() => {
    fitBoundsToRoute(routePath, flightTracks);
  }, [routePath, flightTracks]);

  const goToBase = useCallback(
    (baseId) => {
      const base = missionMarkers.bases.find((b) => b.id === baseId);
      if (base)
        map.flyTo({ center: [base.longitude, base.latitude], zoom: Math.max(map.getZoom(), 16) });
    },
    [missionMarkers.bases],
  );

  // One playback time per route/device, defaulting to that route's start.
  const [playbackTimes, setPlaybackTimes] = useState({});

  const handlePlaybackChange = useCallback((trackId, value) => {
    setPlaybackTimes((prev) => ({ ...prev, [trackId]: value }));
  }, []);

  const playbackPositions = useMemo(
    () =>
      routeTracks.flatMap((track) => {
        const time = playbackTimes[track.id] ?? track.startTime;
        const position = nearestPosition(track.positions, time);
        if (!position) return [];
        return [
          {
            id: `playback-${track.deviceId}`,
            deviceId: track.deviceId,
            longitude: position.longitude,
            latitude: position.latitude,
            fixTime: position.fixTime,
            course: position.course,
          },
        ];
      }),
    [routeTracks, playbackTimes],
  );

  return (
    <div>
      <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
        Mapa de mission
      </Typography>
      <div style={{ width: '100%', height: '500px', position: 'relative' }}>
        <MapView>
          <MapMissions filteredDeviceId={-1} routes={routePath} />
          <MapFlightPath tracks={flightTracks} />
          <MapMarkers markers={missionMarkers} />
          <MapPositions positions={playbackPositions} showStatus={false} />
        </MapView>
        <Paper square elevation={3} className={classes.missionMapOverlay}>
          <TabContext value={tabValue}>
            <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
              <TabList onChange={onTabChange} aria-label="lab API tabs example">
                <Tab label="mission" value="1" />
                <Tab label="Planning" value="2" />
                <Tab label="Playback" value="3" />
              </TabList>
            </Box>
            <TabPanel value="1" sx={{ p: 1 }}>
              {routePath && (
                <RoutesList
                  mission={missions.mission}
                  setmission={noop}
                  setScrool={noop}
                  NoEdit={true}
                />
              )}
            </TabPanel>
            <TabPanel value="2" sx={{ p: 1 }}>
              {missions.task && (
                <>
                  <SelectField
                    emptyValue={null}
                    fullWidth
                    disabled
                    label="objetive"
                    value={missions.task.case}
                    onChange={noop}
                    endpoint="/api/planning/missionstype"
                    keyGetter={(it) => it.id}
                    titleGetter={(it) => it.name}
                  />
                  <Accordion>
                    <AccordionSummary expandIcon={<ExpandMore />}>
                      <Typography>Interest elements</Typography>
                    </AccordionSummary>
                    <AccordionDetails className={classes.details}>
                      {missions.task.locations && <SelectList Data={missions.task.locations} />}
                    </AccordionDetails>
                  </Accordion>
                  <Divider />
                  <Accordion>
                    <AccordionSummary expandIcon={<ExpandMore />}>
                      <Typography>Devices</Typography>
                    </AccordionSummary>
                    <AccordionDetails className={classes.details}>
                      {dataMission && dataParam && (
                        <BaseSettings
                          data={dataMission}
                          markers={missionMarkers}
                          param={dataParam}
                          setData={noop}
                          goToBase={goToBase}
                        />
                      )}
                    </AccordionDetails>
                  </Accordion>
                </>
              )}
            </TabPanel>
            <TabPanel value="3" sx={{ p: 1 }}>
              {routeTracks.length === 0 && (
                <Typography variant="body2" color="text.secondary">
                  No hay posiciones registradas para reproducir.
                </Typography>
              )}
              {routeTracks.map((track) => (
                <Box key={track.id} sx={{ px: 1, pb: 2 }}>
                  <Typography variant="subtitle2">
                    {devices[track.deviceId]?.name ?? track.deviceId}
                  </Typography>
                  <Slider
                    min={track.startTime}
                    max={track.endTime}
                    value={playbackTimes[track.id] ?? track.startTime}
                    onChange={(event, value) => handlePlaybackChange(track.id, value)}
                    valueLabelDisplay="auto"
                    valueLabelFormat={(value) => formatTime(value, 'seconds')}
                  />
                </Box>
              ))}
            </TabPanel>
          </TabContext>
        </Paper>
      </div>
    </div>
  );
};

export default MissionMapPanel;
