import React, { Fragment, useState } from 'react';
import { useDispatch } from 'react-redux';
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

import ExpandMore from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import DeleteIcon from '@mui/icons-material/Delete';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import { missionActions } from '../store';

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

const WaypointRouteList = ({ routeIndex, indexWp, waypoint, idleVel, expandWp, setExpandWp, onAddWaypoint }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const [expanded_ac, setExpanded_ac] = useState(false);
  const [newactionmenu, setnewactionmenu] = useState(true);
  const [newactionid, setnewactionid] = useState(0);

  const handleChange_wp = (panel) => (event, isExpanded) => {
    setExpandWp(isExpanded ? panel : false);
  };

  const handleWaypointFieldChange = (field, value) => {
    dispatch(missionActions.updateWaypoint({ routeIndex, wpIndex: indexWp, field, value }));
  };

  const handlePositionChange = (posIndex, value) => {
    dispatch(missionActions.updateWaypointPosIndex({ routeIndex, wpIndex: indexWp, posIndex, value: +value }));
  };

  const handleActionChange = (actionKey, value) => {
    dispatch(missionActions.updateWaypointAction({ routeIndex, wpIndex: indexWp, actionKey, value }));
  };

  async function handleAddAction() {
    let command;

    const response = await fetch('/api/category/actions/dji_M210_noetic');
    if (response.ok) {
      command = await response.json();
    } else {
      throw Error(await response.text());
    }

    const selectcmd = command.find((element) => element.id == newactionid);

    const actionValue = selectcmd.param ? 0 : true;
    dispatch(missionActions.addWaypointAction({ routeIndex, wpIndex: indexWp, actionKey: selectcmd.name, value: actionValue }));
    setnewactionmenu(true);
  }

  const handleCopyWp = () => {
    dispatch(missionActions.copyWaypoint({ routeIndex, wpIndex: indexWp }));
  };

  const handleMoveWp = (direction) => {
    dispatch(missionActions.moveWaypointOrder({ routeIndex, wpIndex: indexWp, direction }));
  };

  const handleRemoveWp = () => {
    dispatch(missionActions.deleteWaypoint({ routeIndex, wpIndex: indexWp }));
  };

  const handleRemoveAction = (actionKey) => {
    dispatch(missionActions.removeWaypointAction({ routeIndex, wpIndex: indexWp, actionKey }));
  };

  const handleChange_ac = (panel) => (event, isExpanded) => {
    setExpanded_ac(isExpanded ? panel : false);
  };

  return (
    <Accordion expanded={expandWp === `WP${indexWp}`} onChange={handleChange_wp(`WP${indexWp}`)}>
      <AccordionSummary expandIcon={<ExpandMore />}>
        <Typography sx={{ width: '33%', flexShrink: 0 }}>{`WP - ${indexWp}`}</Typography>
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => handleMoveWp(1)}>
          <ArrowDownwardIcon />
        </IconButton>
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => handleMoveWp(-1)}>
          <ArrowUpwardIcon />
        </IconButton>
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={handleRemoveWp}>
          <DeleteIcon />
        </IconButton>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {expandWp === `WP${indexWp}` && (
          <>
            <Box
              component="form"
              sx={{
                '& .MuiTextField-root': { m: 1 },
              }}
            >
              <div>
                <Typography variant="subtitle1" style={{ display: 'inline' }}>
                  Position
                </Typography>
              </div>
              <TextField
                required
                label="Latitud "
                type="number"
                sx={{ width: '15ch' }}
                variant="standard"
                inputProps={{
                  maxLength: 16,
                  step: 0.0001,
                }}
                value={waypoint.pos ? waypoint.pos[0] : 0}
                onChange={(e) => handlePositionChange(0, e.target.value)}
              />
              <TextField
                required
                label="Longitud "
                type="number"
                variant="standard"
                sx={{ width: '15ch' }}
                inputProps={{
                  maxLength: 16,
                  step: 0.0001,
                }}
                value={waypoint.pos ? waypoint.pos[1] : 0}
                onChange={(e) => handlePositionChange(1, e.target.value)}
              />
              <TextField
                required
                label="altura "
                type="number"
                variant="standard"
                sx={{ width: '7ch' }}
                value={waypoint.pos ? waypoint.pos[2] : 0}
                onChange={(e) => handlePositionChange(2, e.target.value)}
              />
            </Box>
            <Box
              component="form"
              sx={{
                '& .MuiTextField-root': { m: 1 },
              }}
            >
              <TextField
                required
                label="Speed "
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                value={waypoint.speed ?? idleVel ?? 3}
                onChange={(e) => handleWaypointFieldChange('speed', +e.target.value)}
              />

              <TextField
                required
                label="YAW "
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                value={waypoint.yaw ?? 0}
                onChange={(e) => handleWaypointFieldChange('yaw', +e.target.value)}
              />

              <TextField
                required
                label="Gimbal "
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                value={waypoint.gimbal ?? 0}
                onChange={(e) => handleWaypointFieldChange('gimbal', +e.target.value)}
              />
            </Box>
            <Accordion expanded={expanded_ac === 'wp ' + indexWp} onChange={handleChange_ac('wp ' + indexWp)}>
              <AccordionSummary expandIcon={<ExpandMore />}>
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
                            onChange={(e) => handleActionChange(action_key, e.target.value)}
                          />
                        </div>
                        <IconButton
                          sx={{
                            py: 0,
                            pr: 2,
                            marginLeft: 'auto',
                          }}
                          onClick={() => handleRemoveAction(action_key)}
                          className={classes.negative}
                        >
                          <DeleteIcon />
                        </IconButton>
                      </div>
                      <Divider />
                    </Fragment>
                  ))}
                <Box textAlign="center">
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
                      <Typography variant="subtitle1">Tipo de acccion a añadir</Typography>
                      <SelectField
                        emptyValue={null}
                        fullWidth={true}
                        value={newactionid}
                        onChange={(e) => setnewactionid(e.target.value)}
                        endpoint={'/api/category/actions/dji_M210_noetic'}
                        keyGetter={(it) => it.id}
                        titleGetter={(it) => it.description}
                      />
                      <div>
                        <Button onClick={() => setnewactionmenu(true)}>Cancel</Button>
                        <Button onClick={handleAddAction}>Add</Button>
                      </div>
                    </div>
                  )}
                </Box>
              </AccordionDetails>
            </Accordion>
            <Box textAlign="center">
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
                onClick={handleCopyWp}
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
