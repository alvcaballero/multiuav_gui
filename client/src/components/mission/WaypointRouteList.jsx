import React, { Fragment, useState } from 'react';
import {
  Divider,
  Box,
  Button,
  IconButton,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  Stack,
} from '@mui/material';
import AddCircleIcon from '@mui/icons-material/AddCircle';
import { makeStyles } from 'tss-react/mui';

import { map } from '../../map/core/mapInstance';
import ExpandMore from '@mui/icons-material/ExpandMore';
import SelectField from '../../shared/components/SelectField';
import DeleteIcon from '@mui/icons-material/Delete';
import MyLocationIcon from '@mui/icons-material/MyLocation';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import useWaypoint from './useWaypoint';

const useStyles = makeStyles()((theme) => ({
  list: {
    maxHeight: '100%',
    overflow: 'auto',
  },
  icon: {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  attributeName: {
    display: 'inline-block',
    width: '40%',
    textAlign: 'left',
    verticalAlign: 'middle',
  },
  attributeValue: {
    display: 'inline-block',
    width: '58%',
  },
  actionValue: {
    display: 'inline-block',
    width: '40%',
  },
}));

// Renders a waypoint action's value input driven by its catalog payload metadata
// (type/unit/min/max/step). Number inputs clamp to [min,max] on blur so the value
// committed to the mission is always firmware-valid. Falls back to a plain field
// when no payload metadata is available (boolean flag actions or metadata pending).
const ActionValueField = ({ payload, value, onCommit, disabled = false }) => {
  const [draft, setDraft] = useState(value ?? payload?.default ?? 0);

  React.useEffect(() => {
    setDraft(value ?? payload?.default ?? 0);
  }, [value, payload]);

  if (!payload || payload.type !== 'number') {
    return (
      <TextField
        size="small"
        variant="standard"
        disabled={disabled}
        sx={{ flexGrow: 1 }}
        value={draft}
        onChange={(e) => setDraft(e.target.value)}
        onBlur={() => onCommit(draft)}
      />
    );
  }

  const { min, max, step, unit } = payload;
  const clamp = (n) => {
    let v = Number(n);
    if (Number.isNaN(v)) v = payload.default ?? 0;
    if (min != null && v < min) v = min;
    if (max != null && v > max) v = max;
    return v;
  };

  return (
    <TextField
      size="small"
      type="number"
      variant="standard"
      disabled={disabled}
      sx={{ flexGrow: 1 }}
      value={draft}
      slotProps={{
        htmlInput: { min, max, step },
        input: unit ? { endAdornment: unit } : undefined,
      }}
      onChange={(e) => setDraft(e.target.value)}
      onBlur={() => onCommit(clamp(draft))}
    />
  );
};

// Renders a per-waypoint param (speed, gimbal, mode_turn...) from its catalog def
// (type/min/max/step/unit/options). Number → clamped numeric field; select → dropdown.
// Which params appear is driven entirely by the category profile's waypoint_params.
const WaypointParamField = ({ def, value, onCommit, disabled = false }) => {
  const initial = value ?? def.default ?? 0;
  const [draft, setDraft] = useState(initial);

  React.useEffect(() => {
    setDraft(value ?? def.default ?? 0);
  }, [value, def]);

  if (def.type === 'select') {
    return (
      <TextField
        select
        label={def.name}
        variant="standard"
        disabled={disabled}
        sx={{ width: '13ch' }}
        value={value ?? def.default ?? ''}
        onChange={(e) => onCommit(Number(e.target.value))}
        slotProps={{ select: { native: true } }}
      >
        {(def.options ?? []).map((opt) => (
          <option key={opt.value} value={opt.value}>
            {opt.name}
          </option>
        ))}
      </TextField>
    );
  }

  const clamp = (n) => {
    let v = Number(n);
    if (Number.isNaN(v)) v = def.default ?? 0;
    if (def.min != null && v < def.min) v = def.min;
    if (def.max != null && v > def.max) v = def.max;
    return v;
  };

  return (
    <TextField
      required
      label={def.name}
      type="number"
      variant="standard"
      disabled={disabled}
      sx={{ width: '13ch' }}
      value={draft}
      slotProps={{
        htmlInput: { min: def.min, max: def.max, step: def.step },
        input: def.unit ? { endAdornment: def.unit } : undefined,
      }}
      onChange={(e) => setDraft(e.target.value)}
      onBlur={() => onCommit(clamp(draft))}
    />
  );
};

const WaypointRouteList = ({
  routeIndex,
  indexWp,
  waypoint,
  idleVel,
  uavType,
  expanded,
  setExpanded,
  onAddWaypoint,
  NoEdit = false,
}) => {
  const { classes } = useStyles();
  const { updateField, updatePos, updateAction, removeAction, addAction, copy, remove, move } =
    useWaypoint(routeIndex, indexWp, uavType);

  const [newactionmenu, setNewactionmenu] = useState(true);
  const [newactionid, setNewactionid] = useState(0);
  const [posInput, setPosInput] = useState({ lat: '', lon: '', alt: '' });
  // Action metadata (payload type/range/unit) keyed by action name, from the catalog
  const [actionDefs, setActionDefs] = useState({});
  // Per-waypoint param defs (speed, gimbal, mode_turn...) from the catalog profile
  const [wpParams, setWpParams] = useState([]);

  const wpKey = `r${routeIndex}-wp${indexWp}`;
  const isOpen = expanded === wpKey;

  // Fetch the rich action catalog + per-waypoint params for this category once open
  React.useEffect(() => {
    if (!isOpen || !uavType) return;
    const controller = new AbortController();
    fetch(`/api/category/actions/${uavType}`, { signal: controller.signal })
      .then((r) => (r.ok ? r.json() : []))
      .then((defs) => {
        setActionDefs(Object.fromEntries(defs.map((d) => [d.name, d])));
      })
      .catch(() => {});
    fetch(`/api/category/waypointparams/${uavType}`, { signal: controller.signal })
      .then((r) => (r.ok ? r.json() : []))
      .then((defs) => {
        setWpParams(defs);
      })
      .catch(() => {});
    return () => {
      controller.abort();
    };
  }, [isOpen, uavType]);

  // Keep posInput in sync when waypoint is updated externally (e.g. map drag)
  React.useEffect(() => {
    if (!isOpen) return;
    setPosInput({
      lat: waypoint.pos?.[0] ?? 0,
      lon: waypoint.pos?.[1] ?? 0,
      alt: waypoint.pos?.[2] ?? 0,
    });
  }, [waypoint.pos, isOpen]);

  const handleChange_wp = (_, isExpanded) => {
    // closing the WP keeps the parent route open
    setExpanded(isExpanded ? wpKey : `r${routeIndex}`);
  };

  const handlePositionCommit = () => {
    updatePos([+posInput.lat, +posInput.lon, +posInput.alt]);
  };

  return (
    <Accordion expanded={isOpen} onChange={handleChange_wp}>
      <AccordionSummary expandIcon={<ExpandMore />} component="div">
        <Typography sx={{ width: '33%', flexShrink: 0 }}>{`WP - ${indexWp}`}</Typography>
        {!NoEdit && (
          <>
            <IconButton
              sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
              onClick={(e) => {
                e.stopPropagation();
                move(1);
              }}
            >
              <ArrowDownwardIcon />
            </IconButton>
            <IconButton
              sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
              onClick={(e) => {
                e.stopPropagation();
                move(-1);
              }}
            >
              <ArrowUpwardIcon />
            </IconButton>
          </>
        )}
        <IconButton
          sx={{ py: 0, pr: 0, flexShrink: 0 }}
          onClick={(e) => {
            e.stopPropagation();
            map.flyTo({
              center: [waypoint.pos[1], waypoint.pos[0]],
              zoom: Math.max(map.getZoom(), 16),
            });
          }}
        >
          <MyLocationIcon />
        </IconButton>
        {!NoEdit && (
          <IconButton
            sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
            onClick={(e) => {
              e.stopPropagation();
              remove();
            }}
          >
            <DeleteIcon />
          </IconButton>
        )}
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {isOpen && (
          <>
            <Box component="form" sx={{ '& .MuiTextField-root': { m: 1 } }}>
              <div>
                <Typography variant="subtitle1" style={{ display: 'inline' }}>
                  Position
                </Typography>
              </div>
              <TextField
                required
                disabled={NoEdit}
                label="Latitud"
                type="number"
                sx={{ width: '15ch' }}
                variant="standard"
                slotProps={{ htmlInput: { step: 0.0001 } }}
                value={posInput.lat}
                onChange={(e) => setPosInput((p) => ({ ...p, lat: e.target.value }))}
                onBlur={handlePositionCommit}
                onKeyDown={(e) => e.key === 'Enter' && handlePositionCommit()}
              />
              <TextField
                required
                disabled={NoEdit}
                label="Longitud"
                type="number"
                variant="standard"
                sx={{ width: '15ch' }}
                slotProps={{ htmlInput: { step: 0.0001 } }}
                value={posInput.lon}
                onChange={(e) => setPosInput((p) => ({ ...p, lon: e.target.value }))}
                onBlur={handlePositionCommit}
                onKeyDown={(e) => e.key === 'Enter' && handlePositionCommit()}
              />
              <TextField
                required
                disabled={NoEdit}
                label="Altura"
                type="number"
                variant="standard"
                sx={{ width: '7ch' }}
                value={posInput.alt}
                onChange={(e) => setPosInput((p) => ({ ...p, alt: e.target.value }))}
                onBlur={handlePositionCommit}
                onKeyDown={(e) => e.key === 'Enter' && handlePositionCommit()}
              />
            </Box>
            <Box component="form" sx={{ '& .MuiTextField-root': { m: 1 } }}>
              {/* YAW is intrinsic to every aerial robot — always shown. */}
              <TextField
                required
                disabled={NoEdit}
                label="YAW"
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                defaultValue={waypoint.yaw ?? 0}
                onBlur={(e) => updateField('yaw', +e.target.value)}
              />
              {/* Speed / Gimbal / Turn (etc.) come from the catalog per-waypoint params. */}
              {wpParams.map((p) => (
                <WaypointParamField
                  key={p.id}
                  // speed inherits the route's idle_vel as its shown default
                  def={p.id === 'speed' && idleVel != null ? { ...p, default: idleVel } : p}
                  value={waypoint[p.id]}
                  onCommit={(v) => updateField(p.id, v)}
                  disabled={NoEdit}
                />
              ))}
            </Box>
            <Box>
              <Stack direction="row" alignItems="center" spacing={0.5} sx={{ mb: 0.5 }}>
                <Typography variant="subtitle1">Actions</Typography>
                {!NoEdit && (
                  <IconButton
                    size="small"
                    onClick={() => setNewactionmenu((v) => !v)}
                    title="Add action"
                  >
                    <AddCircleIcon fontSize="small" />
                  </IconButton>
                )}
              </Stack>
              {!NoEdit && !newactionmenu && (
                <Stack direction="row" alignItems="center" spacing={1} sx={{ mb: 1 }}>
                  <Box sx={{ flexGrow: 1, minWidth: 0 }}>
                    <SelectField
                      label="Action"
                      fullWidth
                      emptyValue={null}
                      value={newactionid}
                      onChange={(e) => setNewactionid(e.target.value)}
                      endpoint={`/api/category/actions/${uavType}`}
                      keyGetter={(it) => it.id}
                      titleGetter={(it) => it.description}
                    />
                  </Box>
                  <Button
                    size="small"
                    variant="contained"
                    onClick={() => addAction(newactionid, () => setNewactionmenu(true))}
                  >
                    Add
                  </Button>
                  <Button size="small" onClick={() => setNewactionmenu(true)}>
                    Cancel
                  </Button>
                </Stack>
              )}
              {waypoint.action &&
                Object.keys(waypoint.action).map((action_key, index_ac) => (
                  <Fragment key={'fragment-action-' + index_ac}>
                    <Stack direction="row" alignItems="center" spacing={1} sx={{ py: 0.5 }}>
                      <Typography variant="body2" sx={{ width: '40%', flexShrink: 0 }}>
                        {action_key}
                      </Typography>
                      <ActionValueField
                        payload={actionDefs[action_key]?.payload}
                        value={waypoint.action[action_key]}
                        onCommit={(v) => updateAction(action_key, v)}
                        disabled={NoEdit}
                      />
                      {!NoEdit && (
                        <IconButton size="small" onClick={() => removeAction(action_key)}>
                          <DeleteIcon fontSize="small" />
                        </IconButton>
                      )}
                    </Stack>
                    <Divider />
                  </Fragment>
                ))}
            </Box>
            {!NoEdit && (
              <Box sx={{ textAlign: 'center' }}>
                <Button
                  variant="contained"
                  size="large"
                  sx={{ width: '60%', flexShrink: 0 }}
                  style={{ marginTop: '15px' }}
                  onClick={() => onAddWaypoint(routeIndex, indexWp)}
                >
                  Add new Waypoint
                </Button>
                <Button
                  variant="contained"
                  color="secondary"
                  size="large"
                  sx={{ width: '30%', flexShrink: 0 }}
                  style={{ marginTop: '15px' }}
                  onClick={copy}
                >
                  Copy
                </Button>
              </Box>
            )}
          </>
        )}
      </AccordionDetails>
    </Accordion>
  );
};

export default WaypointRouteList;
