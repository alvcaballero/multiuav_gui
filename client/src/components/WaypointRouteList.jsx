import React, { Fragment, useEffect, useState } from 'react';
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
import makeStyles from '@mui/styles/makeStyles';
import ExpandMore from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import DeleteIcon from '@mui/icons-material/Delete';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import MyLocationIcon from '@mui/icons-material/MyLocation';

const useStyles = makeStyles((theme) => ({
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

const WaypointRouteList = ({ mission, setmission, index, indexWp, waypoint, expandWp, AddnewWp, setExpandWp }) => {
  const classes = useStyles();
  const [expanded_ac, setExpanded_ac] = useState(false);
  const [newactionmenu, setnewactionmenu] = useState(true);
  const [newactionid, setnewactionid] = useState(0);

  const handleChange_wp = (panel) => (event, isExpanded) => {
    setExpandWp(isExpanded ? panel : false);
  };

  async function setnewaction(indexRoute, indexWp) {
    let command;

    const response = await fetch('/api/category/actions/dji_M210_noetic');
    if (response.ok) {
      command = await response.json();
    } else {
      throw Error(await response.text());
    }

    let selectcmd = command.find((element) => element.id == newactionid);

    let auxroute = JSON.parse(JSON.stringify(mission.route));
    if (!auxroute[indexRoute]['wp'][indexWp].hasOwnProperty('action')) {
      auxroute[indexRoute]['wp'][indexWp]['action'] = {};
    }
    if (selectcmd.param) {
      auxroute[indexRoute]['wp'][indexWp]['action'][selectcmd.name] = 0;
    } else {
      auxroute[indexRoute]['wp'][indexWp]['action'][selectcmd.name] = true;
    }
    setmission({ ...mission, route: auxroute });
    setnewactionmenu(true);
  }
  const CopyWp = (index, indexWp) => {
    const auxRoute = JSON.parse(JSON.stringify(mission.route));
    const auxWaypoint = auxRoute[index].wp[indexWp];
    auxRoute[index].wp.splice(indexWp + 1, 0, auxWaypoint);
    setmission({ ...mission, route: auxRoute });
  };
  const MoveWp = (index, indexWp, direction) => {
    const auxRoute = JSON.parse(JSON.stringify(mission.route));
    const auxWaypoint = auxRoute[index].wp[indexWp];
    auxRoute[index].wp.splice(indexWp, 1);
    if (indexWp + direction < 0) {
      auxRoute[index].wp.push(auxWaypoint);
    } else {
      auxRoute[index].wp.splice(indexWp + direction, 0, auxWaypoint);
    }
    setmission({ ...mission, route: auxRoute });
  };

  const RemoveWp = (indexRoute, indexWp) => {
    console.log('remove wp' + indexRoute + '-' + indexWp);
    let auxroute = [...mission.route];
    let auxwaypoint = [...mission.route[indexRoute]['wp']];
    auxwaypoint.splice(indexWp, 1);
    let aux = { ...auxroute[indexRoute], wp: auxwaypoint };
    auxroute[indexRoute] = aux;
    setmission({ ...mission, route: auxroute });
  };
  const Removing_action = (indexRoute, indexWp, action) => {
    console.log('remove action' + indexRoute + '-' + indexWp + '-' + action);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    delete auxroute[indexRoute]['wp'][indexWp]['action'][action];
    setmission({ ...mission, route: auxroute });
  };
  const handleChange_ac = (panel) => (event, isExpanded) => {
    setExpanded_ac(isExpanded ? panel : false);
  };

  return (
    <Accordion expanded={expandWp === `WP${indexWp}`} onChange={handleChange_wp(`WP${indexWp}`)}>
      <AccordionSummary expandIcon={<ExpandMore />}>
        <Typography sx={{ width: '33%', flexShrink: 0 }}>{`WP - ${indexWp}`}</Typography>
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => MoveWp(index, indexWp, 1)}>
          <ArrowDownwardIcon />
        </IconButton>
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => MoveWp(index, indexWp, -1)}>
          <ArrowUpwardIcon />
        </IconButton>
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => RemoveWp(index, indexWp)}>
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
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == mission.route[index] ? (copiedrt.wp[indexWp]['pos'][0] = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
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
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == mission.route[index] ? (copiedrt.wp[indexWp]['pos'][1] = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
              />
              <TextField
                required
                label="altura "
                type="number"
                variant="standard"
                sx={{ width: '7ch' }}
                value={waypoint.pos ? waypoint.pos[2] : 0}
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == mission.route[index] ? (copiedrt.wp[indexWp]['pos'][2] = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
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
                value={waypoint.speed ? waypoint.speed : mission.route[index].attributes.idle_vel}
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      mission.route[index] ? (copiedrt.wp[indexWp]['speed'] = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
              />

              <TextField
                required
                label="YAW "
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                value={waypoint.yaw ? waypoint.yaw : 0}
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == mission.route[index] ? (copiedrt.wp[indexWp]['yaw'] = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
              />

              <TextField
                required
                label="Gimbal "
                type="number"
                variant="standard"
                sx={{ width: '13ch' }}
                value={waypoint.gimbal ? waypoint.gimbal : 0}
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == mission.route[index] ? (copiedrt.wp[indexWp]['gimbal'] = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
              />
            </Box>
            <Accordion expanded={expanded_ac === 'wp ' + indexWp} onChange={handleChange_ac('wp ' + indexWp)}>
              <AccordionSummary expandIcon={<ExpandMore />}>
                <Typography sx={{ width: '33%', flexShrink: 0 }}>Actions</Typography>
              </AccordionSummary>
              <AccordionDetails className={classes.details}>
                {waypoint.action &&
                  React.Children.toArray(
                    Object.keys(waypoint.action).map((action_key, index_ac, list_ac) => (
                      <Fragment key={'fragment-action-' + index_ac}>
                        <div>
                          <Typography variant="subtitle1" className={classes.attributeName}>
                            {action_key}
                          </Typography>
                          <div className={classes.actionValue}>
                            <TextField
                              required
                              fullWidth={true}
                              value={waypoint.action[action_key] ? waypoint.action[action_key] : 0}
                              onChange={(e) =>
                                setmission({
                                  ...mission,
                                  route: mission.route.map((rt) => {
                                    let copiedrt = JSON.parse(JSON.stringify(rt));
                                    rt == mission.route[index]
                                      ? (copiedrt.wp[indexWp]['action'][action_key] = e.target.value)
                                      : (copiedrt = rt);
                                    return copiedrt;
                                  }),
                                })
                              }
                            />
                          </div>
                          <IconButton
                            sx={{
                              py: 0,
                              pr: 2,
                              marginLeft: 'auto',
                            }}
                            onClick={() => Removing_action(index, indexWp, action_key)}
                            className={classes.negative}
                          >
                            <DeleteIcon />
                          </IconButton>
                        </div>
                        <Divider />
                      </Fragment>
                    ))
                  )}
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
                        <Button onClick={() => setnewaction(index, indexWp)}>Add</Button>
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
                onClick={() => AddnewWp(index, indexWp)}
              >
                Add new Waypoint
              </Button>
              <Button
                variant="contained"
                color="secondary"
                size="large"
                sx={{ width: '30%', flexShrink: 0 }}
                style={{ marginTop: '15px' }}
                onClick={() => CopyWp(index, indexWp)}
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
