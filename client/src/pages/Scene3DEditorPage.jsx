import React, { useState, useCallback } from 'react';
import { useDispatch, useSelector } from 'react-redux';

import {
  Box,
  Paper,
  Tab,
  Typography,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  IconButton,
  Divider,
  Chip,
  List,
  ListItemButton,
  ListItemText,
  ListItemIcon,
  Tooltip,
  Button,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  ToggleButton,
  ToggleButtonGroup,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
} from '@mui/material';
import { TabPanel, TabList, TabContext } from '@mui/lab';
import { makeStyles } from 'tss-react/mui';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import MyLocationIcon from '@mui/icons-material/MyLocation';
import HandymanIcon from '@mui/icons-material/Handyman';
import RotateRightIcon from '@mui/icons-material/RotateRight';
import OpenWithIcon from '@mui/icons-material/OpenWith';

import maplibregl from 'maplibre-gl';

import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import Scene3DCanvas from '../scene3d/Scene3DCanvas';
import { sessionActions, missionActions } from '../store';
import { useCatch } from '../reactHelper';

// ---------------------------------------------------------------------------
// Helpers — mirrors convertion.js but inverse: XYZ (meters) → lat/lon
// ---------------------------------------------------------------------------

/** Local XYZ (East, North, Up in metres) relative to origin → { lat, lng, alt } */
const xyzToLatLon = (origin, x, y, z) => {
  const ref = maplibregl.MercatorCoordinate.fromLngLat(
    { lng: origin.lng, lat: origin.lat },
    origin.alt ?? 0,
  );
  const mpu = ref.meterInMercatorCoordinateUnits();
  const mCoord = new maplibregl.MercatorCoordinate(ref.x + x * mpu, ref.y - y * mpu, ref.z);
  const lngLat = mCoord.toLngLat();
  return { lat: lngLat.lat, lng: lngLat.lng, alt: z };
};

/** { lat, lng, alt } → local XYZ (East, North, Up in metres) relative to origin */
const latLonToXyz = (origin, lat, lng, alt) => {
  const ref = maplibregl.MercatorCoordinate.fromLngLat(
    { lng: origin.lng, lat: origin.lat },
    origin.alt ?? 0,
  );
  const pt = maplibregl.MercatorCoordinate.fromLngLat({ lng, lat }, alt ?? 0);
  const mpu = ref.meterInMercatorCoordinateUnits();
  return {
    x: +((pt.x - ref.x) / mpu).toFixed(2),
    y: +((ref.y - pt.y) / mpu).toFixed(2),
    z: +(alt ?? 0),
  };
};

// ---------------------------------------------------------------------------
// Styles
// ---------------------------------------------------------------------------
const SIDEBAR_WIDTH = 420;

const useStyles = makeStyles()((theme) => ({
  root: { height: '100vh', overflow: 'hidden' },
  canvas: {
    position: 'fixed',
    top: theme.dimensions.navbarHeight,
    left: SIDEBAR_WIDTH,
    right: 0,
    bottom: 0,
  },
  sidebar: {
    position: 'fixed',
    left: 0,
    top: theme.dimensions.navbarHeight,
    width: SIDEBAR_WIDTH,
    height: `calc(100% - ${theme.dimensions.navbarHeight})`,
    display: 'flex',
    flexDirection: 'column',
    zIndex: 4,
    overflowY: 'auto',
  },
  xyzRow: {
    display: 'flex',
    gap: theme.spacing(1),
    alignItems: 'center',
    flexWrap: 'wrap',
  },
  coordField: { width: '90px' },
  sectionHeader: {
    padding: theme.spacing(1, 2),
    background: theme.palette.action.hover,
  },
  chip: { fontFamily: 'monospace', fontSize: '0.7rem' },
  accordionDetails: { display: 'flex', flexDirection: 'column', gap: theme.spacing(1.5) },
}));

// ---------------------------------------------------------------------------
// Sub-component: one XYZ editor row
// ---------------------------------------------------------------------------
const XYZEditor = ({ label, xyz, onChange, showAlt = true }) => {
  const { classes } = useStyles();
  const [local, setLocal] = useState({ x: xyz.x, y: xyz.y, z: xyz.z });

  // keep local in sync when parent changes (e.g. tab switch)
  React.useEffect(() => {
    setLocal({ x: xyz.x, y: xyz.y, z: xyz.z });
  }, [xyz.x, xyz.y, xyz.z]);

  const commit = useCallback(() => onChange(local), [local, onChange]);

  const handleKey = (e) => {
    if (e.key === 'Enter') commit();
  };

  return (
    <Box className={classes.xyzRow}>
      {label && (
        <Typography variant="caption" sx={{ minWidth: 60 }}>
          {label}
        </Typography>
      )}
      <TextField
        className={classes.coordField}
        label="X (E) m"
        type="number"
        size="small"
        variant="outlined"
        value={local.x}
        onChange={(e) => setLocal((p) => ({ ...p, x: +e.target.value }))}
        onBlur={commit}
        onKeyDown={handleKey}
        slotProps={{ htmlInput: { step: 0.5 } }}
      />
      <TextField
        className={classes.coordField}
        label="Y (N) m"
        type="number"
        size="small"
        variant="outlined"
        value={local.y}
        onChange={(e) => setLocal((p) => ({ ...p, y: +e.target.value }))}
        onBlur={commit}
        onKeyDown={handleKey}
        slotProps={{ htmlInput: { step: 0.5 } }}
      />
      {showAlt && (
        <TextField
          className={classes.coordField}
          label="Z (Alt) m"
          type="number"
          size="small"
          variant="outlined"
          value={local.z}
          onChange={(e) => setLocal((p) => ({ ...p, z: +e.target.value }))}
          onBlur={commit}
          onKeyDown={handleKey}
          slotProps={{ htmlInput: { step: 0.5 } }}
        />
      )}
    </Box>
  );
};

// ---------------------------------------------------------------------------
// Heading editor (degrees from North, 0-360)
// ---------------------------------------------------------------------------
const HeadingEditor = ({ heading, onChange }) => {
  const [local, setLocal] = useState(heading);

  React.useEffect(() => setLocal(heading), [heading]);

  const commit = useCallback(() => {
    const clamped = ((+local % 360) + 360) % 360;
    setLocal(clamped);
    onChange(clamped);
  }, [local, onChange]);

  return (
    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mt: 0.5 }}>
      <TextField
        label="Heading (° from N)"
        type="number"
        size="small"
        variant="outlined"
        value={local}
        onChange={(e) => setLocal(e.target.value)}
        onBlur={commit}
        onKeyDown={(e) => e.key === 'Enter' && commit()}
        slotProps={{ htmlInput: { min: 0, max: 360, step: 5 } }}
        sx={{ width: 150 }}
      />
      <Typography variant="caption" color="text.secondary">
        0=N · 90=E · 180=S · 270=W
      </Typography>
    </Box>
  );
};

// ---------------------------------------------------------------------------
// Tab 1 — Origin
// ---------------------------------------------------------------------------
const OriginTab = () => {
  const dispatch = useDispatch();
  const origin = useSelector((state) => state.session.scene3d.origin);
  const range = useSelector((state) => state.session.scene3d.range);
  const bases = useSelector((state) => state.session.markers.bases);

  const [latLocal, setLatLocal] = useState(String(origin.lat));
  const [lngLocal, setLngLocal] = useState(String(origin.lng));
  const [altLocal, setAltLocal] = useState(String(origin.alt ?? 0));
  const [rangeLocal, setRangeLocal] = useState(String(range));

  // Keep fields in sync when origin changes externally (e.g. SelectDevice3D)
  React.useEffect(() => {
    setLatLocal(String(origin.lat));
    setLngLocal(String(origin.lng));
    setAltLocal(String(origin.alt ?? 0));
  }, [origin.lat, origin.lng, origin.alt]);

  const applyOrigin = useCallback(
    (lat, lng, alt) => {
      dispatch(sessionActions.updateScene3dOrigin({ lat: +lat, lng: +lng, alt: +alt }));
    },
    [dispatch],
  );

  const commitOrigin = useCallback(
    () => applyOrigin(latLocal, lngLocal, altLocal),
    [applyOrigin, latLocal, lngLocal, altLocal],
  );

  const handleKey = (e) => {
    if (e.key === 'Enter') commitOrigin();
  };

  const setOriginFromBase = useCallback(
    (base) => {
      const alt = origin.alt ?? 0;
      setLatLocal(String(base.latitude));
      setLngLocal(String(base.longitude));
      setAltLocal(String(alt));
      applyOrigin(base.latitude, base.longitude, alt);
    },
    [applyOrigin, origin.alt],
  );

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2, p: 2 }}>
      <Typography variant="body2" color="text.secondary">
        The origin is the reference point (0, 0, 0) of the 3D scene. All XYZ positions are expressed
        in metres relative to it (X = East, Y = North, Z = Up).
      </Typography>

      <TextField
        label="Latitude"
        type="number"
        size="small"
        value={latLocal}
        onChange={(e) => setLatLocal(e.target.value)}
        onBlur={commitOrigin}
        onKeyDown={handleKey}
        slotProps={{ htmlInput: { step: 0.0001 } }}
      />
      <TextField
        label="Longitude"
        type="number"
        size="small"
        value={lngLocal}
        onChange={(e) => setLngLocal(e.target.value)}
        onBlur={commitOrigin}
        onKeyDown={handleKey}
        slotProps={{ htmlInput: { step: 0.0001 } }}
      />
      <TextField
        label="Altitude (m)"
        type="number"
        size="small"
        value={altLocal}
        onChange={(e) => setAltLocal(e.target.value)}
        onBlur={commitOrigin}
        onKeyDown={handleKey}
        slotProps={{ htmlInput: { step: 1 } }}
      />

      <Box sx={{ p: 1, bgcolor: 'action.hover', borderRadius: 1 }}>
        <Typography variant="caption" sx={{ fontFamily: 'monospace' }}>
          Active → ({origin.lat.toFixed(6)}, {origin.lng.toFixed(6)}, {origin.alt ?? 0} m)
        </Typography>
      </Box>

      <Divider />

      {/* ---- Set origin from base ---- */}
      {bases.length > 0 && (
        <>
          <Typography variant="subtitle2">Set origin from base</Typography>
          <List
            dense
            disablePadding
            sx={{ border: 1, borderColor: 'divider', borderRadius: 1, overflow: 'hidden' }}
          >
            {bases.map((base, idx) => {
              const isActive =
                Math.abs(origin.lat - base.latitude) < 1e-7 &&
                Math.abs(origin.lng - base.longitude) < 1e-7;
              return (
                <Tooltip
                  key={base.id ?? idx}
                  title={`lat ${base.latitude?.toFixed(6)}  lon ${base.longitude?.toFixed(6)}`}
                  placement="right"
                >
                  <ListItemButton
                    selected={isActive}
                    onClick={() => setOriginFromBase(base)}
                    divider={idx < bases.length - 1}
                  >
                    <ListItemIcon sx={{ minWidth: 32 }}>
                      <MyLocationIcon fontSize="small" color={isActive ? 'primary' : 'action'} />
                    </ListItemIcon>
                    <ListItemText
                      primary={base.name || `Base ${idx}`}
                      secondary={`(${base.latitude?.toFixed(5)}, ${base.longitude?.toFixed(5)})`}
                      primaryTypographyProps={{ variant: 'body2' }}
                      secondaryTypographyProps={{
                        variant: 'caption',
                        sx: { fontFamily: 'monospace' },
                      }}
                    />
                  </ListItemButton>
                </Tooltip>
              );
            })}
          </List>
          <Divider />
        </>
      )}

      <Typography variant="caption" color="text.secondary">
        Scene range: objects beyond ±range metres are culled.
      </Typography>
      <TextField
        label="Range (m)"
        type="number"
        size="small"
        value={rangeLocal}
        onChange={(e) => setRangeLocal(e.target.value)}
        onBlur={() => applyOrigin(latLocal, lngLocal, altLocal)}
        slotProps={{ htmlInput: { step: 100, min: 100 } }}
      />
    </Box>
  );
};

// ---------------------------------------------------------------------------
// Tab 2 — Markers (bases + elements)
// ---------------------------------------------------------------------------
const MarkersTab = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const origin = useSelector((state) => state.session.scene3d.origin);
  const markers = useSelector((state) => state.session.markers);
  const range = useSelector((state) => state.session.scene3d.range);

  const inRange = useCallback(
    (lat, lng) => {
      const xyz = latLonToXyz(origin, lat, lng, 0);
      return Math.abs(xyz.x) < range && Math.abs(xyz.y) < range;
    },
    [origin, range],
  );

  const [expandedBase, setExpandedBase] = useState(false);
  const [expandedElem, setExpandedElem] = useState(false);
  const [saveStatus, setSaveStatus] = useState('idle'); // idle | saving | ok | error

  const saveMarkers = useCatch(async () => {
    setSaveStatus('saving');
    const planning = { markersbase: markers.bases, elements: markers.elements };
    const response = await fetch('api/planning/setDefault', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(planning),
    });
    if (!response.ok) throw new Error(await response.text());
    setSaveStatus('ok');
    setTimeout(() => setSaveStatus('idle'), 2000);
  });

  // -- Bases --
  const updateBase = useCallback(
    (index, xyz) => {
      const { lat, lng } = xyzToLatLon(origin, xyz.x, xyz.y, xyz.z);
      const newBases = structuredClone(markers.bases);
      newBases[index].latitude = lat;
      newBases[index].longitude = lng;
      dispatch(sessionActions.updateMarker({ ...markers, bases: newBases }));
    },
    [dispatch, markers, origin],
  );

  // -- Elements (items inside each group) --
  const updateElement = useCallback(
    (groupIdx, itemIdx, xyz) => {
      const { lat, lng } = xyzToLatLon(origin, xyz.x, xyz.y, xyz.z);
      const newElements = structuredClone(markers.elements);
      newElements[groupIdx].items[itemIdx].latitude = lat;
      newElements[groupIdx].items[itemIdx].longitude = lng;
      dispatch(sessionActions.updateMarker({ ...markers, elements: newElements }));
    },
    [dispatch, markers, origin],
  );

  const updateElementHeading = useCallback(
    (groupIdx, itemIdx, heading) => {
      const newElements = structuredClone(markers.elements);
      newElements[groupIdx].items[itemIdx].heading = Math.min(360, Math.max(0, +heading));
      dispatch(sessionActions.updateMarker({ ...markers, elements: newElements }));
    },
    [dispatch, markers],
  );

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1, p: 1 }}>
      {/* ---- BASES ---- */}
      {(() => {
        const visibleBases = markers.bases.filter((b) => inRange(b.latitude, b.longitude));
        return (
          <>
            <Typography variant="subtitle2" className={classes.sectionHeader}>
              Bases ({visibleBases.length} / {markers.bases.length} in range)
            </Typography>
            {visibleBases.length === 0 && (
              <Typography variant="caption" sx={{ px: 2, color: 'text.secondary' }}>
                No bases within range.
              </Typography>
            )}
            {visibleBases.map((base) => {
              const idx = markers.bases.indexOf(base);
              const xyz = latLonToXyz(origin, base.latitude, base.longitude, 0);
              return (
                <Accordion
                  key={base.id ?? idx}
                  expanded={expandedBase === idx}
                  onChange={(_, open) => setExpandedBase(open ? idx : false)}
                  disableGutters
                >
                  <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                    <Typography variant="body2" sx={{ flexGrow: 1 }} noWrap>
                      {base.name || `Base ${idx}`}
                    </Typography>
                    <Chip
                      className={classes.chip}
                      label={`(${xyz.x}, ${xyz.y})`}
                      size="small"
                      variant="outlined"
                      sx={{ mr: 1 }}
                    />
                  </AccordionSummary>
                  <AccordionDetails className={classes.accordionDetails}>
                    <Typography variant="caption" color="text.secondary">
                      lat {base.latitude?.toFixed(6)} · lon {base.longitude?.toFixed(6)}
                    </Typography>
                    <XYZEditor
                      xyz={xyz}
                      showAlt={false}
                      onChange={(newXyz) => updateBase(idx, newXyz)}
                    />
                  </AccordionDetails>
                </Accordion>
              );
            })}
          </>
        );
      })()}

      <Divider sx={{ my: 1 }} />

      {/* ---- ELEMENTS ---- */}
      {(() => {
        const groupsWithVisible = markers.elements.flatMap((group) => {
          const visibleItems = (group.items ?? []).flatMap((item, iIdx) =>
            inRange(item.latitude, item.longitude) ? [{ item, iIdx }] : [],
          );
          return visibleItems.length > 0 ? [{ group, visibleItems }] : [];
        });

        const totalVisible = groupsWithVisible.reduce(
          (acc, { visibleItems }) => acc + visibleItems.length,
          0,
        );
        const totalAll = markers.elements.reduce((acc, g) => acc + (g.items?.length ?? 0), 0);

        return (
          <>
            <Typography variant="subtitle2" className={classes.sectionHeader}>
              Elements ({totalVisible} / {totalAll} in range)
            </Typography>
            {groupsWithVisible.length === 0 && (
              <Typography variant="caption" sx={{ px: 2, color: 'text.secondary' }}>
                No elements within range.
              </Typography>
            )}
            {groupsWithVisible.map(({ group, visibleItems }) => {
              const realGIdx = markers.elements.indexOf(group);
              return (
                <Accordion
                  key={realGIdx}
                  expanded={expandedElem === realGIdx}
                  onChange={(_, open) => setExpandedElem(open ? realGIdx : false)}
                  disableGutters
                >
                  <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                    <Typography variant="body2" sx={{ flexGrow: 1 }} noWrap>
                      {group.name || `Group ${realGIdx}`}
                      <Typography
                        component="span"
                        variant="caption"
                        color="text.secondary"
                        sx={{ ml: 1 }}
                      >
                        ({group.type}) · {visibleItems.length} / {group.items?.length ?? 0} items
                      </Typography>
                    </Typography>
                  </AccordionSummary>
                  <AccordionDetails className={classes.accordionDetails}>
                    {visibleItems.map(({ item, iIdx }) => {
                      const xyz = latLonToXyz(origin, item.latitude, item.longitude, 0);
                      return (
                        <Box
                          key={item.id ?? iIdx}
                          sx={{ pl: 1, borderLeft: '2px solid', borderColor: 'divider' }}
                        >
                          <Typography variant="caption" color="text.secondary">
                            {item.name || `Item ${iIdx}`} · lat {item.latitude?.toFixed(5)} · lon{' '}
                            {item.longitude?.toFixed(5)}
                          </Typography>
                          <XYZEditor
                            xyz={xyz}
                            showAlt={false}
                            onChange={(newXyz) => updateElement(realGIdx, iIdx, newXyz)}
                          />
                          <HeadingEditor
                            heading={item.heading ?? 0}
                            onChange={(h) => updateElementHeading(realGIdx, iIdx, h)}
                          />
                        </Box>
                      );
                    })}
                  </AccordionDetails>
                </Accordion>
              );
            })}
          </>
        );
      })()}

      <Divider sx={{ mt: 2 }} />
      <Box sx={{ p: 2 }}>
        <Button
          variant="contained"
          fullWidth
          disabled={saveStatus === 'saving'}
          color={saveStatus === 'ok' ? 'success' : 'primary'}
          onClick={saveMarkers}
        >
          {saveStatus === 'saving' && 'Saving…'}
          {saveStatus === 'ok' && 'Saved!'}
          {(saveStatus === 'idle' || saveStatus === 'error') && 'Save markers as default'}
        </Button>
      </Box>
    </Box>
  );
};

// ---------------------------------------------------------------------------
// Transform dialog — rotate (degrees) or translate (metres XYZ)
// ---------------------------------------------------------------------------
const TransformDialog = ({ open, onClose, routes, origin }) => {
  const dispatch = useDispatch();
  const [mode, setMode] = useState('rotate');
  const [routeIndex, setRouteIndex] = useState(-1);
  const [angleDeg, setAngleDeg] = useState('');
  const [dx, setDx] = useState('');
  const [dy, setDy] = useState('');

  const isDisabled =
    mode === 'rotate'
      ? angleDeg === '' || isNaN(parseFloat(angleDeg))
      : dx === '' || dy === '' || isNaN(parseFloat(dx)) || isNaN(parseFloat(dy));

  const handleApply = () => {
    if (mode === 'rotate') {
      dispatch(missionActions.rotateMission({ angleDeg: parseFloat(angleDeg), routeIndex }));
    } else {
      // Convert metres (XYZ) → delta lat/lng using origin as reference
      const ref = maplibregl.MercatorCoordinate.fromLngLat({ lng: origin.lng, lat: origin.lat }, 0);
      const mpu = ref.meterInMercatorCoordinateUnits();
      // X = East → +lng,  Y = North → +lat
      const shifted = new maplibregl.MercatorCoordinate(
        ref.x + parseFloat(dx) * mpu,
        ref.y - parseFloat(dy) * mpu,
        0,
      );
      const shiftedLL = shifted.toLngLat();
      const deltaLat = shiftedLL.lat - origin.lat;
      const deltaLng = shiftedLL.lng - origin.lng;
      dispatch(missionActions.translateMission({ deltaLat, deltaLng, routeIndex }));
    }
    handleClose();
  };

  const handleClose = () => {
    setAngleDeg('');
    setDx('');
    setDy('');
    onClose();
  };

  return (
    <Dialog open={open} onClose={handleClose} maxWidth="xs" fullWidth>
      <DialogTitle>Transform Mission</DialogTitle>
      <DialogContent>
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2, mt: 1 }}>
          <ToggleButtonGroup
            value={mode}
            exclusive
            onChange={(_, val) => val && setMode(val)}
            fullWidth
            size="small"
          >
            <ToggleButton value="rotate">
              <RotateRightIcon sx={{ mr: 1 }} /> Rotate
            </ToggleButton>
            <ToggleButton value="translate">
              <OpenWithIcon sx={{ mr: 1 }} /> Translate
            </ToggleButton>
          </ToggleButtonGroup>

          <FormControl size="small" fullWidth>
            <InputLabel>Route</InputLabel>
            <Select
              value={routeIndex}
              label="Route"
              onChange={(e) => setRouteIndex(e.target.value)}
            >
              <MenuItem value={-1}>All routes</MenuItem>
              {routes.map((route, i) => (
                <MenuItem key={route.id ?? i} value={i}>
                  {route.name || `Route ${i + 1}`}
                </MenuItem>
              ))}
            </Select>
          </FormControl>

          {mode === 'rotate' && (
            <>
              <Typography variant="caption" color="text.secondary">
                Rotation around the centroid. Positive = counter-clockwise.
              </Typography>
              <TextField
                label="Angle (degrees)"
                type="number"
                size="small"
                value={angleDeg}
                onChange={(e) => setAngleDeg(e.target.value)}
                slotProps={{ htmlInput: { step: 1 } }}
                fullWidth
              />
            </>
          )}

          {mode === 'translate' && (
            <>
              <Typography variant="caption" color="text.secondary">
                Offset in metres relative to the scene origin. X = East, Y = North.
              </Typography>
              <TextField
                label="ΔX East (m)"
                type="number"
                size="small"
                value={dx}
                onChange={(e) => setDx(e.target.value)}
                slotProps={{ htmlInput: { step: 1 } }}
                fullWidth
              />
              <TextField
                label="ΔY North (m)"
                type="number"
                size="small"
                value={dy}
                onChange={(e) => setDy(e.target.value)}
                slotProps={{ htmlInput: { step: 1 } }}
                fullWidth
              />
            </>
          )}
        </Box>
      </DialogContent>
      <DialogActions>
        <Button onClick={handleClose}>Cancel</Button>
        <Button onClick={handleApply} variant="contained" disabled={isDisabled}>
          Apply
        </Button>
      </DialogActions>
    </Dialog>
  );
};

// ---------------------------------------------------------------------------
// Tab 3 — Waypoints
// ---------------------------------------------------------------------------
const WaypointsTab = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const origin = useSelector((state) => state.session.scene3d.origin);
  const routes = useSelector((state) => state.mission.route);

  const [expandedRoute, setExpandedRoute] = useState(false);
  const [expandedWp, setExpandedWp] = useState({});
  const [transformOpen, setTransformOpen] = useState(false);

  const updateWpXyz = useCallback(
    (routeIndex, wpIndex, xyz) => {
      const geo = xyzToLatLon(origin, xyz.x, xyz.y, xyz.z);
      dispatch(
        missionActions.updateWaypointPos({
          routeIndex,
          wpIndex,
          pos: [geo.lat, geo.lng, geo.alt],
        }),
      );
    },
    [dispatch, origin],
  );

  const toggleWp = (rIdx, wIdx) => {
    setExpandedWp((prev) => {
      const key = `${rIdx}-${wIdx}`;
      return { ...prev, [key]: !prev[key] };
    });
  };

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1, p: 1 }}>
      {/* Toolbar */}
      <Box sx={{ display: 'flex', justifyContent: 'flex-end', px: 1 }}>
        <Tooltip title="Transform mission (rotate / translate)">
          <span>
            <IconButton
              size="small"
              onClick={() => setTransformOpen(true)}
              disabled={routes.length === 0}
            >
              <HandymanIcon fontSize="small" />
            </IconButton>
          </span>
        </Tooltip>
      </Box>

      {routes.length === 0 && (
        <Typography variant="caption" sx={{ px: 2, color: 'text.secondary' }}>
          No mission loaded.
        </Typography>
      )}
      {routes.map((route, rIdx) => (
        <Accordion
          key={route.id ?? rIdx}
          expanded={expandedRoute === rIdx}
          onChange={(_, open) => setExpandedRoute(open ? rIdx : false)}
          disableGutters
        >
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Typography variant="body2" sx={{ flexGrow: 1 }} noWrap>
              {route.name || `Route ${rIdx}`}
              <Typography component="span" variant="caption" color="text.secondary" sx={{ ml: 1 }}>
                · {route.wp?.length ?? 0} wps · {route.uav || 'unassigned'}
              </Typography>
            </Typography>
          </AccordionSummary>
          <AccordionDetails className={classes.accordionDetails}>
            {(route.wp ?? []).map((wp, wIdx) => {
              const [lat, lng, alt] = wp.pos ?? [0, 0, 0];
              const xyz = latLonToXyz(origin, lat, lng, alt);
              const key = `${rIdx}-${wIdx}`;
              return (
                <Box
                  key={wIdx}
                  sx={{ pl: 1, borderLeft: '2px solid', borderColor: 'primary.light' }}
                >
                  <Box
                    sx={{ display: 'flex', alignItems: 'center', cursor: 'pointer', gap: 1 }}
                    onClick={() => toggleWp(rIdx, wIdx)}
                  >
                    <Chip label={`WP ${wIdx}`} size="small" color="primary" variant="outlined" />
                    <Typography variant="caption" color="text.secondary">
                      ({xyz.x} m, {xyz.y} m, {xyz.z} m)
                    </Typography>
                    <ExpandMoreIcon
                      sx={{
                        ml: 'auto',
                        fontSize: 16,
                        transform: expandedWp[key] ? 'rotate(180deg)' : 'none',
                        transition: 'transform 0.2s',
                      }}
                    />
                  </Box>
                  {expandedWp[key] && (
                    <Box sx={{ mt: 1 }}>
                      <Typography
                        variant="caption"
                        color="text.secondary"
                        sx={{ display: 'block', mb: 0.5 }}
                      >
                        lat {lat.toFixed(6)} · lon {lng.toFixed(6)} · alt {alt} m
                      </Typography>
                      <XYZEditor
                        xyz={xyz}
                        showAlt
                        onChange={(newXyz) => updateWpXyz(rIdx, wIdx, newXyz)}
                      />
                    </Box>
                  )}
                </Box>
              );
            })}
            {(!route.wp || route.wp.length === 0) && (
              <Typography variant="caption" color="text.secondary">
                No waypoints in this route.
              </Typography>
            )}
          </AccordionDetails>
        </Accordion>
      ))}

      <TransformDialog
        open={transformOpen}
        onClose={() => setTransformOpen(false)}
        routes={routes}
        origin={origin}
      />
    </Box>
  );
};

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------
const TABS = { ORIGIN: '1', MARKERS: '2', WAYPOINTS: '3' };

const Scene3DEditorPage = () => {
  const { classes } = useStyles();
  const [tab, setTab] = useState(TABS.ORIGIN);

  return (
    <div className={classes.root}>
      <Navbar />
      <Menu />

      <Scene3DCanvas className={classes.canvas} style={{ position: 'fixed' }} />

      <Paper square elevation={4} className={classes.sidebar}>
        <TabContext value={tab}>
          <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
            <TabList onChange={(_, v) => setTab(v)} aria-label="3D editor tabs">
              <Tab label="Origin" value={TABS.ORIGIN} />
              <Tab label="Markers" value={TABS.MARKERS} />
              <Tab label="Waypoints" value={TABS.WAYPOINTS} />
            </TabList>
          </Box>
          <TabPanel value={TABS.ORIGIN} sx={{ p: 0, overflow: 'auto', flex: 1 }}>
            <OriginTab />
          </TabPanel>
          <TabPanel value={TABS.MARKERS} sx={{ p: 0, overflow: 'auto', flex: 1 }}>
            <MarkersTab />
          </TabPanel>
          <TabPanel value={TABS.WAYPOINTS} sx={{ p: 0, overflow: 'auto', flex: 1 }}>
            <WaypointsTab />
          </TabPanel>
        </TabContext>
      </Paper>
    </div>
  );
};

export default Scene3DEditorPage;
