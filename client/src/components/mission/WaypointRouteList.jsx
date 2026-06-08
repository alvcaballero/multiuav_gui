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
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import { map } from '../../map/core/MapView';
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

const WaypointRouteList = ({ routeIndex, indexWp, waypoint, idleVel, uavType, expanded, setExpanded, onAddWaypoint }) => {
  const { classes } = useStyles();
  const { updateField, updatePos, updateAction, removeAction, addAction, copy, remove, move } = useWaypoint(routeIndex, indexWp, uavType);

  const [expanded_ac, setExpanded_ac] = useState(false);
  const [newactionmenu, setnewactionmenu] = useState(true);
  const [newactionid, setnewactionid] = useState(0);
  const [posInput, setPosInput] = useState({ lat: '', lon: '', alt: '' });

  const wpKey = `r${routeIndex}-wp${indexWp}`;
  const isOpen = expanded === wpKey;

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

  const handleChange_ac = (panel) => (event, isExpanded) => {
    setExpanded_ac(isExpanded ? panel : false);
  };

  return (
    <Accordion expanded={isOpen} onChange={handleChange_wp}>
      <AccordionSummary expandIcon={<ExpandMore />} component="div">
        <Typography sx={{ width: '33%', flexShrink: 0 }}>{`WP - ${indexWp}`}</Typography>
        <IconButton
          sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
          onClick={(e) => { e.stopPropagation(); move(1); }}
        >
          <ArrowDownwardIcon />
        </IconButton>
        <IconButton
          sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
          onClick={(e) => { e.stopPropagation(); move(-1); }}
        >
          <ArrowUpwardIcon />
        </IconButton>
        <IconButton
          sx={{ py: 0, pr: 0, flexShrink: 0 }}
          onClick={(e) => {
            e.stopPropagation();
            map.flyTo({ center: [waypoint.pos[1], waypoint.pos[0]], zoom: Math.max(map.getZoom(), 16) });
          }}
        >
          <MyLocationIcon />
        </IconButton>
        <IconButton
          sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
          onClick={(e) => { e.stopPropagation(); remove(); }}
        >
          <DeleteIcon />
        </IconButton>
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
              <TextField
                required
                label="Speed"
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                defaultValue={waypoint.speed ?? idleVel ?? 3}
                onBlur={(e) => updateField('speed', +e.target.value)}
              />
              <TextField
                required
                label="YAW"
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                defaultValue={waypoint.yaw ?? 0}
                onBlur={(e) => updateField('yaw', +e.target.value)}
              />
              <TextField
                required
                label="Gimbal"
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                defaultValue={waypoint.gimbal ?? 0}
                onBlur={(e) => updateField('gimbal', +e.target.value)}
              />
            </Box>
            <Accordion expanded={expanded_ac === 'wp ' + indexWp} onChange={handleChange_ac('wp ' + indexWp)}>
              <AccordionSummary component="div" expandIcon={<ExpandMore />}>
                <Typography sx={{ width: '33%', flexShrink: 0 }}>Actions</Typography>
              </AccordionSummary>
              <AccordionDetails className={classes.details}>
                {waypoint.action &&
                  Object.keys(waypoint.action).map((action_key, index_ac) => (
                    <Fragment key={'fragment-action-' + index_ac}>
                      <div>
                        <Typography variant="subtitle1" className={classes.attributeName}>
                          {action_key}
                        </Typography>
                        <div className={classes.actionValue}>
                          <TextField
                            required
                            fullWidth={true}
                            value={waypoint.action[action_key] ?? 0}
                            onChange={(e) => updateAction(action_key, e.target.value)}
                          />
                        </div>
                        <IconButton
                          sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                          onClick={() => removeAction(action_key)}
                        >
                          <DeleteIcon />
                        </IconButton>
                      </div>
                      <Divider />
                    </Fragment>
                  ))}
                <Box sx={{ textAlign: 'center' }}>
                  {newactionmenu ? (
                    <Button
                      variant="contained"
                      size="large"
                      sx={{ width: '80%', flexShrink: 0 }}
                      style={{ marginTop: '15px' }}
                      onClick={() => setnewactionmenu(false)}
                    >
                      Add new action
                    </Button>
                  ) : (
                    <div>
                      <Typography variant="subtitle1">Tipo de acción a añadir</Typography>
                      <SelectField
                        emptyValue={null}
                        fullWidth={true}
                        value={newactionid}
                        onChange={(e) => setnewactionid(e.target.value)}
                        endpoint={`/api/category/actions/${uavType}`}
                        keyGetter={(it) => it.id}
                        titleGetter={(it) => it.description}
                      />
                      <div>
                        <Button onClick={() => setnewactionmenu(true)}>Cancel</Button>
                        <Button onClick={() => addAction(newactionid, () => setnewactionmenu(true))}>Add</Button>
                      </div>
                    </div>
                  )}
                </Box>
              </AccordionDetails>
            </Accordion>
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
          </>
        )}
      </AccordionDetails>
    </Accordion>
  );
};

export default WaypointRouteList;
