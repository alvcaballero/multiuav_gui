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

const WaypointRouteList = ({
  mission,
  setmission,
  index,
  index_wp,
  waypoint,
  expand_wp,
  AddnewWp,
}) => {
  const classes = useStyles();
  const [expanded_ac, setExpanded_ac] = useState(false);
  const [newactionmenu, setnewactionmenu] = useState(true);
  const [newactionid, setnewactionid] = useState(0);
  const [expanded_wp, setExpanded_wp] = useState(false);
  //const waypoint = mission.route[index]["wp"][index_wp];

  useEffect(() => {
    setExpanded_wp(expand_wp);
  }, [expand_wp]);

  const handleChange_wp = (panel) => (event, isExpanded) => {
    setExpanded_wp(isExpanded ? panel : false);
  };

  async function setnewaction(index_route, index_wp) {
    let command;

    const response = await fetch('/api/category/actions/dji_M210_noetic');
    if (response.ok) {
      command = await response.json();
    } else {
      throw Error(await response.text());
    }

    let selectcmd = command.find((element) => element.id == newactionid);

    let auxroute = JSON.parse(JSON.stringify(mission.route));
    if (!auxroute[index_route]['wp'][index_wp].hasOwnProperty('action')) {
      auxroute[index_route]['wp'][index_wp]['action'] = {};
    }
    if (selectcmd.param) {
      auxroute[index_route]['wp'][index_wp]['action'][selectcmd.name] = 0;
    } else {
      auxroute[index_route]['wp'][index_wp]['action'][selectcmd.name] = true;
    }
    setmission({ ...mission, route: auxroute });
    setnewactionmenu(true);
  }

  const Removing_wp = (index_route, index_wp) => {
    console.log('remove wp' + index_route + '-' + index_wp);
    let auxroute = [...mission.route];
    let auxwaypoint = [...mission.route[index_route]['wp']];
    auxwaypoint.splice(index_wp, 1);
    let aux = { ...auxroute[index_route], wp: auxwaypoint };
    auxroute[index_route] = aux;
    setmission({ ...mission, route: auxroute });
  };
  const Removing_action = (index_route, index_wp, action) => {
    console.log('remove action' + index_route + '-' + index_wp + '-' + action);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    delete auxroute[index_route]['wp'][index_wp]['action'][action];
    setmission({ ...mission, route: auxroute });
  };
  const handleChange_ac = (panel) => (event, isExpanded) => {
    setExpanded_ac(isExpanded ? panel : false);
  };
  const handleChange_acnew = (panel) => (event, isExpanded) => {
    setnewactionmenu(false);
  };

  return (
    <Accordion
      expanded={expanded_wp === 'wp ' + index_wp}
      onChange={handleChange_wp('wp ' + index_wp)}
    >
      <AccordionSummary expandIcon={<ExpandMore />}>
        <Typography sx={{ width: '33%', flexShrink: 0 }}>{'WP ' + index_wp}</Typography>
        <IconButton
          sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
          onClick={() => Removing_wp(index, index_wp)}
        >
          <DeleteIcon />
        </IconButton>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        <Box
          component='form'
          sx={{
            '& .MuiTextField-root': { m: 1 },
          }}
        >
          <div>
            <Typography variant='subtitle1' style={{ display: 'inline' }}>
              Position
            </Typography>
          </div>
          <TextField
            required
            label='Latitud '
            type='number'
            sx={{ width: '15ch' }}
            variant='standard'
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
                  rt == mission.route[index]
                    ? (copiedrt.wp[index_wp]['pos'][0] = +e.target.value)
                    : (copiedrt = rt);
                  return copiedrt;
                }),
              })
            }
          />
          <TextField
            required
            label='Longitud '
            type='number'
            variant='standard'
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
                  rt == mission.route[index]
                    ? (copiedrt.wp[index_wp]['pos'][1] = +e.target.value)
                    : (copiedrt = rt);
                  return copiedrt;
                }),
              })
            }
          />
          <TextField
            required
            label='altura '
            type='number'
            variant='standard'
            sx={{ width: '7ch' }}
            value={waypoint.pos ? waypoint.pos[2] : 0}
            onChange={(e) =>
              setmission({
                ...mission,
                route: mission.route.map((rt) => {
                  let copiedrt = JSON.parse(JSON.stringify(rt));
                  rt == mission.route[index]
                    ? (copiedrt.wp[index_wp]['pos'][2] = +e.target.value)
                    : (copiedrt = rt);
                  return copiedrt;
                }),
              })
            }
          />
        </Box>

        <TextField
          required
          label='Speed '
          type='number'
          variant='standard'
          value={waypoint.speed ? waypoint.speed : mission.route[index].attributes.idle_vel}
          onChange={(e) =>
            setmission({
              ...mission,
              route: mission.route.map((rt) => {
                let copiedrt = JSON.parse(JSON.stringify(rt));
                mission.route[index]
                  ? (copiedrt.wp[index_wp]['speed'] = +e.target.value)
                  : (copiedrt = rt);
                return copiedrt;
              }),
            })
          }
        />

        <TextField
          required
          label='YAW '
          type='number'
          variant='standard'
          value={waypoint.yaw ? waypoint.yaw : 0}
          onChange={(e) =>
            setmission({
              ...mission,
              route: mission.route.map((rt) => {
                let copiedrt = JSON.parse(JSON.stringify(rt));
                rt == mission.route[index]
                  ? (copiedrt.wp[index_wp]['yaw'] = +e.target.value)
                  : (copiedrt = rt);
                return copiedrt;
              }),
            })
          }
        />

        <TextField
          required
          label='Gimbal '
          type='number'
          variant='standard'
          value={waypoint.gimbal ? waypoint.gimbal : 0}
          onChange={(e) =>
            setmission({
              ...mission,
              route: mission.route.map((rt) => {
                let copiedrt = JSON.parse(JSON.stringify(rt));
                rt == mission.route[index]
                  ? (copiedrt.wp[index_wp]['gimbal'] = +e.target.value)
                  : (copiedrt = rt);
                return copiedrt;
              }),
            })
          }
        />

        <Accordion
          expanded={expanded_ac === 'wp ' + index_wp}
          onChange={handleChange_ac('wp ' + index_wp)}
        >
          <AccordionSummary expandIcon={<ExpandMore />}>
            <Typography sx={{ width: '33%', flexShrink: 0 }}>Actions</Typography>
          </AccordionSummary>
          <AccordionDetails className={classes.details}>
            {waypoint.action &&
              React.Children.toArray(
                Object.keys(waypoint.action).map((action_key, index_ac, list_ac) => (
                  <Fragment key={'fragment-action-' + index_ac}>
                    <div>
                      <Typography variant='subtitle1' className={classes.attributeName}>
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
                                  ? (copiedrt.wp[index_wp]['action'][action_key] = e.target.value)
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
                        onClick={() => Removing_action(index, index_wp, action_key)}
                        className={classes.negative}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </div>
                    <Divider></Divider>
                  </Fragment>
                ))
              )}
            <Box textAlign='center'>
              {newactionmenu ? (
                <Button
                  variant='contained'
                  size='large'
                  sx={{ width: '80%', flexShrink: 0 }}
                  style={{ marginTop: '15px' }}
                  onClick={handleChange_acnew('wp ' + index_wp)}
                >
                  Add new action
                </Button>
              ) : (
                <div>
                  <Typography variant='subtitle1'>Tipo de acccion a añadir</Typography>
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
                    <Button onClick={() => setnewaction(index, index_wp)}>Add</Button>
                  </div>
                </div>
              )}
            </Box>
          </AccordionDetails>
        </Accordion>
        <Box textAlign='center'>
          <Button
            variant='contained'
            size='large'
            sx={{ width: '80%', flexShrink: 0 }}
            style={{ marginTop: '15px' }}
            onClick={() => AddnewWp(index, index_wp)}
          >
            Add new Waypoint
          </Button>
        </Box>
      </AccordionDetails>
    </Accordion>
  );
};

export default WaypointRouteList;
