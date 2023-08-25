import React, { Fragment, useEffect, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import makeStyles from "@mui/styles/makeStyles";
import {
  Divider,
  Box,
  Button,
  IconButton,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
} from "@mui/material";
import ArrowRight from "@mui/icons-material/ArrowRight";
import KeyboardArrowDown from "@mui/icons-material/KeyboardArrowDown";
import Typography from "@mui/material/Typography";
import ExpandLess from "@mui/icons-material/ExpandLess";
import ExpandMore from "@mui/icons-material/ExpandMore";
import SelectField from "../common/components/SelectField";
import DeleteIcon from "@mui/icons-material/Delete";
import AddIcon from "@mui/icons-material/Add";
import MyLocationIcon from "@mui/icons-material/MyLocation";
import { Padding, Tune } from "@mui/icons-material";
import { missionActions } from "../store"; // here update device action with position of uav for update in map
import { colors } from "../Mapview/preloadImages";
import { map } from "../Mapview/Mapview";

//import { geofencesActions } from '../store';
//import CollectionActions from '../settings/components/CollectionActions';
//import { useCatchCallback } from '../reactHelper';
// https://dev.to/shareef/how-to-work-with-arrays-in-reactjs-usestate-4cmi

const useStyles = makeStyles((theme) => ({
  list: {
    maxHeight: "100%",
    overflow: "auto",
  },
  icon: {
    width: "25px",
    height: "25px",
    filter: "brightness(0) invert(1)",
  },
  details: {
    display: "flex",
    flexDirection: "column",
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  attributeName: {
    display: "inline-block",
    width: "40%",
    textAlign: "left",
    verticalAlign: "middle",
  },
  attributeValue: {
    display: "inline-block",
    width: "58%",
  },
  actionValue: {
    display: "inline-block",
    width: "40%",
  },
}));

const RoutesList = ({ mission, setmission, setScrool }) => {
  const classes = useStyles();
  const Mission_route = useSelector((state) => state.mission);
  const selectwp = useSelector((state) => state.mission.selectpoint);
  const dispatch = useDispatch();
  const [init, setinit] = useState(false);
  const [open_routes, setOpen_routes] = useState(true);
  const [expanded_route, setExpanded_route] = useState(false);
  const [expanded_wp, setExpanded_wp] = useState(false);
  const [expanded_ac, setExpanded_ac] = useState(false);
  const [add_mission, setaddMission] = useState(true);
  const [newactionmenu, setnewactionmenu] = useState(true);
  const [newactionid, setnewactionid] = useState(0);

  useEffect(() => {
    setmission(Mission_route);
    console.log("otro use effect");
    Mission_route.route.length > 0 ? setaddMission(false) : setaddMission(true);
    setinit(true);
  }, [Mission_route]);

  useEffect(() => {
    if (init) {
      console.log("update mission");
      console.log(mission);
      dispatch(missionActions.reloadMission(mission.route));
    }
    mission.route.length > 0 ? setaddMission(false) : setaddMission(true);
  }, [mission]);

  useEffect(() => {
    console.log(expanded_route);
    setExpanded_route("Rute " + selectwp.route_id);
    setExpanded_wp("wp " + selectwp.id);
    setScrool(500 + selectwp.route_id * 50 + selectwp.id * 50);
  }, [selectwp]);

  const AddnewMission = () => {
    let auxmission = { name: "new Mission", description: "", route: [] };
    auxmission.route.push({ name: "", uav: "", attributes: {}, wp: [] });
    setmission(auxmission);
    setaddMission(false);
    console.log("add new mission");
  };
  const AddnewRoute = () => {
    let auxroute = [...mission.route];
    auxroute.push({ name: "", uav: "", attributes: {}, wp: [] });
    setmission({ ...mission, route: auxroute });
    console.log("add new route");
  };
  const AddnewWp = (index_route, index_wp) => {
    console.log("add new wp" + index_route);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    let center = map.getCenter();
    if (index_wp < 0) {
      auxroute[index_route].wp.push({
        pos: [center.lat, center.lng, 5],
        action: {},
      });
    } else {
      auxroute[index_route].wp.splice(index_wp, 0, {
        pos: [center.lat, center.lng, 5],
        action: {},
      });
    }

    setmission({ ...mission, route: auxroute });
  };

  async function setnewaction(index_route, index_wp) {
    let command;

    const response = await fetch("/api/mission/actions/dji");
    if (response.ok) {
      command = await response.json();
    } else {
      throw Error(await response.text());
    }
    
    let selectcmd = command.find((element) => element.id == newactionid);

    let auxroute = JSON.parse(JSON.stringify(mission.route));
    if(!auxroute[index_route]["wp"][index_wp].hasOwnProperty("action")){
      auxroute[index_route]["wp"][index_wp]["action"]={}
    }
    if (selectcmd.param) {
      auxroute[index_route]["wp"][index_wp]["action"][selectcmd.name] = 0;
    } else {
      auxroute[index_route]["wp"][index_wp]["action"][selectcmd.name] = true;
    }
    setmission({ ...mission, route: auxroute });
    setnewactionmenu(true);
  }

  const Removing_route = (index_route) => {
    console.log("remove route" + index_route);
    let auxroute = [...mission.route];
    console.log(auxroute);
    auxroute.splice(index_route, 1);
    setmission({ ...mission, route: auxroute });
  };
  const Removing_wp = (index_route, index_wp) => {
    console.log("remove wp" + index_route + "-" + index_wp);
    let auxroute = [...mission.route];
    let auxwaypoint = [...mission.route[index_route]["wp"]];
    auxwaypoint.splice(index_wp, 1);
    let aux = { ...auxroute[index_route], wp: auxwaypoint };
    auxroute[index_route] = aux;
    setmission({ ...mission, route: auxroute });
  };
  const Removing_action = (index_route, index_wp, action) => {
    console.log("remove action" + index_route + "-" + index_wp + "-" + action);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    delete auxroute[index_route]["wp"][index_wp]["action"][action];
    setmission({ ...mission, route: auxroute });
  };

  const handleChange_route = (panel) => (event, isExpanded) => {
    setExpanded_route(isExpanded ? panel : false);
  };
  const handleChange_wp = (panel) => (event, isExpanded) => {
    setExpanded_wp(isExpanded ? panel : false);
  };
  const handleChange_ac = (panel) => (event, isExpanded) => {
    setExpanded_ac(isExpanded ? panel : false);
  };
  const handleChange_acnew = (panel) => (event, isExpanded) => {
    setnewactionmenu(false);
  };

  return (
    <Fragment>
      {add_mission ? (
        <Box textAlign="center">
          <Button
            variant="contained"
            size="large"
            sx={{ width: "80%", flexShrink: 0 }}
            style={{ marginTop: "15px" }}
            onClick={AddnewMission}
          >
            Create New Mission
          </Button>
        </Box>
      ) : (
        <Fragment>
          <Box
            style={{
              display: "flex",
              gap: "10px",
              flexDirection: "column",
              margin: "20px",
            }}
          >
            <TextField
              required
              label="Name Mission"
              variant="standard"
              value={mission.name ? mission.name : " "}
              onChange={(event) =>
                setmission({ ...mission, name: event.target.value })
              }
            />
            <TextField
              required
              label="Description of mission"
              variant="standard"
              value={mission.description ? mission.description : " "}
              onChange={(event) =>
                setmission({ ...mission, description: event.target.value })
              }
            />
            {React.Children.toArray(
              Object.values(mission.route).map((item_route, index, list) => (
                <Fragment key={"fragment-route-" + index}>
                  <Accordion
                    expanded={expanded_route === "Rute " + index}
                    onChange={handleChange_route("Rute " + index)}
                  >
                    <AccordionSummary expandIcon={<ExpandMore />}>
                      <Typography
                        sx={{
                          width: "33%",
                          flexShrink: 0,
                          color: colors[index],
                        }}
                      >
                        {"Rute " + index}
                      </Typography>
                      <Typography sx={{ color: "text.secondary" }}>
                        {item_route.name + "- " + item_route.uav}
                      </Typography>
                      <IconButton
                        sx={{ py: 0, pr: 2, marginLeft: "auto" }}
                        onClick={() => Removing_route(index)}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </AccordionSummary>
                    <AccordionDetails className={classes.details}>
                      <TextField
                        required
                        label="Route Name"
                        variant="standard"
                        value={item_route.name ? item_route.name : ""}
                        onChange={(e) =>
                          setmission({
                            ...mission,
                            route: mission.route.map((rt) =>
                              rt == mission.route[index]
                                ? { ...item_route, name: e.target.value }
                                : rt
                            ),
                          })
                        }
                      />

                      <TextField
                        required
                        label="UAV id"
                        variant="standard"
                        value={item_route.uav ? item_route.uav : "uav_"}
                        onChange={(e) =>
                          setmission({
                            ...mission,
                            route: mission.route.map((rt) =>
                              rt == mission.route[index]
                                ? { ...item_route, uav: e.target.value }
                                : rt
                            ),
                          })
                        }
                      />

                      <SelectField
                        emptyValue={null}
                        value={item_route.uav_type ? item_route.uav_type : ""}
                        onChange={(e) =>
                          setmission({
                            ...mission,
                            route: mission.route.map((rt) =>
                              rt == mission.route[index]
                                ? { ...item_route, uav_type: e.target.value }
                                : rt
                            ),
                          })
                        }
                        endpoint="/api/devices/type"
                        keyGetter={(it) => it}
                        titleGetter={(it) => it}
                        label={"Type UAV mission"}
                        style={{ display: "inline", width: "200px" }}
                      />

                      <Accordion
                        expanded={expanded_wp === "wp " + "-1"}
                        onChange={handleChange_wp("wp " + "-1")}
                      >
                        <AccordionSummary expandIcon={<ExpandMore />}>
                          <Typography sx={{ width: "53%", flexShrink: 0 }}>
                            {" "}
                            Route Attributes
                          </Typography>
                        </AccordionSummary>
                        <AccordionDetails className={classes.details}>
                          {item_route.attributes && (
                            <Fragment
                              key={"fragment-route-atri-" + item_route.id}
                            >
                              <div>
                                <Typography
                                  variant="subtitle1"
                                  className={classes.attributeName}
                                >
                                  Speed idle:{" "}
                                </Typography>
                                <TextField
                                  fullWidth
                                  required
                                  type="number"
                                  className={classes.attributeValue}
                                  value={
                                    item_route.attributes.idle_vel
                                      ? item_route.attributes.idle_vel
                                      : 1.85
                                  }
                                  onChange={(e) =>
                                    setmission({
                                      ...mission,
                                      route: mission.route.map((rt) => {
                                        let copiedrt = JSON.parse(
                                          JSON.stringify(rt)
                                        );
                                        rt == mission.route[index]
                                          ? (copiedrt.attributes.idle_vel =
                                              +e.target.value)
                                          : (copiedrt = rt);
                                        return copiedrt;
                                      }),
                                    })
                                  }
                                />
                              </div>
                              <div>
                                <Typography
                                  variant="subtitle1"
                                  className={classes.attributeName}
                                >
                                  {" "}
                                  landing mode{" "}
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_landing
                                        ? item_route.attributes.mode_landing
                                        : 0
                                    }
                                    onChange={(e) =>
                                      setmission({
                                        ...mission,
                                        route: mission.route.map((rt) => {
                                          let copiedrt = JSON.parse(
                                            JSON.stringify(rt)
                                          );
                                          rt === mission.route[index]
                                            ? (copiedrt.attributes.mode_landing =
                                                e.target.value)
                                            : (copiedrt = rt);
                                          return copiedrt;
                                        }),
                                      })
                                    }
                                    endpoint={
                                      "/api/mission/atributesparam/dji/mode_landing"
                                    }
                                    keyGetter={(it) => it.id}
                                    titleGetter={(it) => it.name}
                                  />
                                </div>
                              </div>
                              <div>
                                <Typography
                                  variant="subtitle1"
                                  className={classes.attributeName}
                                >
                                  Yaw mode
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_yaw
                                        ? item_route.attributes.mode_yaw
                                        : 0
                                    }
                                    onChange={(e) =>
                                      setmission({
                                        ...mission,
                                        route: mission.route.map((rt) => {
                                          let copiedrt = JSON.parse(
                                            JSON.stringify(rt)
                                          );
                                          rt === mission.route[index]
                                            ? (copiedrt.attributes.mode_yaw =
                                                e.target.value)
                                            : (copiedrt = rt);
                                          return copiedrt;
                                        }),
                                      })
                                    }
                                    endpoint={
                                      "/api/mission/atributesparam/dji/mode_yaw"
                                    }
                                    keyGetter={(it) => it.id}
                                    titleGetter={(it) => it.name}
                                  />
                                </div>
                              </div>
                              <div>
                                <Typography
                                  variant="subtitle1"
                                  className={classes.attributeName}
                                >
                                  {" "}
                                  Gimbal mode{" "}
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_gimbal
                                        ? item_route.attributes.mode_gimbal
                                        : 0
                                    }
                                    onChange={(e) =>
                                      setmission({
                                        ...mission,
                                        route: mission.route.map((rt) => {
                                          let copiedrt = JSON.parse(
                                            JSON.stringify(rt)
                                          );
                                          rt === mission.route[index]
                                            ? (copiedrt.attributes.mode_gimbal =
                                                e.target.value)
                                            : (copiedrt = rt);
                                          return copiedrt;
                                        }),
                                      })
                                    }
                                    endpoint={
                                      "/api/mission/atributesparam/dji/mode_gimbal"
                                    }
                                    keyGetter={(it) => it.id}
                                    titleGetter={(it) => it.name}
                                  />
                                </div>
                              </div>
                              <div>
                                <Typography
                                  variant="subtitle1"
                                  className={classes.attributeName}
                                >
                                  {" "}
                                  Trace mode:{" "}
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_trace
                                        ? item_route.attributes.mode_trace
                                        : 0
                                    }
                                    onChange={(e) =>
                                      setmission({
                                        ...mission,
                                        route: mission.route.map((rt) => {
                                          let copiedrt = JSON.parse(
                                            JSON.stringify(rt)
                                          );
                                          rt === mission.route[index]
                                            ? (copiedrt.attributes.mode_trace =
                                                e.target.value)
                                            : (copiedrt = rt);
                                          return copiedrt;
                                        }),
                                      })
                                    }
                                    endpoint={
                                      "/api/mission/atributesparam/dji/mode_trace"
                                    }
                                    keyGetter={(it) => it.id}
                                    titleGetter={(it) => it.name}
                                  />
                                </div>
                              </div>
                            </Fragment>
                          )}
                        </AccordionDetails>
                      </Accordion>

                      <Typography variant="subtitle1">Waypoints</Typography>
                      {React.Children.toArray(
                        Object.values(item_route.wp).map(
                          (waypoint, index_wp, list_wp) => (
                            <Accordion
                              expanded={expanded_wp === "wp " + index_wp}
                              onChange={handleChange_wp("wp " + index_wp)}
                            >
                              <AccordionSummary expandIcon={<ExpandMore />}>
                                <Typography
                                  sx={{ width: "33%", flexShrink: 0 }}
                                >
                                  {"WP " + index_wp}
                                </Typography>
                                <IconButton
                                  sx={{ py: 0, pr: 2, marginLeft: "auto" }}
                                  onClick={() => Removing_wp(index, index_wp)}
                                >
                                  <DeleteIcon />
                                </IconButton>
                              </AccordionSummary>
                              <AccordionDetails className={classes.details}>
                                <Box
                                  component="form"
                                  sx={{
                                    "& .MuiTextField-root": { m: 1 },
                                  }}
                                >
                                  <div>
                                    <Typography
                                      variant="subtitle1"
                                      style={{ display: "inline" }}
                                    >
                                      Position
                                    </Typography>
                                  </div>
                                  <TextField
                                    required
                                    label="Latitud "
                                    type="number"
                                    sx={{ width: "15ch" }}
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
                                          let copiedrt = JSON.parse(
                                            JSON.stringify(rt)
                                          );
                                          rt == mission.route[index]
                                            ? (copiedrt.wp[index_wp]["pos"][0] =
                                            +e.target.value)
                                            : (copiedrt = rt);
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
                                    sx={{ width: "15ch" }}
                                    inputProps={{
                                      maxLength: 16,
                                      step: 0.0001,
                                    }}
                                    value={waypoint.pos ? waypoint.pos[1] : 0}
                                    onChange={(e) =>
                                      setmission({
                                        ...mission,
                                        route: mission.route.map((rt) => {
                                          let copiedrt = JSON.parse(
                                            JSON.stringify(rt)
                                          );
                                          rt == mission.route[index]
                                            ? (copiedrt.wp[index_wp]["pos"][1] =
                                            +e.target.value)
                                            : (copiedrt = rt);
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
                                    sx={{ width: "7ch" }}
                                    value={waypoint.pos ? waypoint.pos[2] : 0}
                                    onChange={(e) =>
                                      setmission({
                                        ...mission,
                                        route: mission.route.map((rt) => {
                                          let copiedrt = JSON.parse(
                                            JSON.stringify(rt)
                                          );
                                          rt == mission.route[index]
                                            ? (copiedrt.wp[index_wp]["pos"][2] =
                                                +e.target.value)
                                            : (copiedrt = rt);
                                          return copiedrt;
                                        }),
                                      })
                                    }
                                  />
                                </Box>

                                <TextField
                                  required
                                  label="YAW "
                                  type="number"
                                  variant="standard"
                                  value={waypoint.yaw ? waypoint.yaw : 0}
                                  onChange={(e) =>
                                    setmission({
                                      ...mission,
                                      route: mission.route.map((rt) => {
                                        let copiedrt = JSON.parse(
                                          JSON.stringify(rt)
                                        );
                                        rt == mission.route[index]
                                          ? (copiedrt.wp[index_wp]["yaw"] =
                                          +e.target.value)
                                          : (copiedrt = rt);
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
                                  value={waypoint.gimbal ? waypoint.gimbal : 0}
                                  onChange={(e) =>
                                    setmission({
                                      ...mission,
                                      route: mission.route.map((rt) => {
                                        let copiedrt = JSON.parse(
                                          JSON.stringify(rt)
                                        );
                                        rt == mission.route[index]
                                          ? (copiedrt.wp[index_wp]["gimbal"] =
                                          +e.target.value)
                                          : (copiedrt = rt);
                                        return copiedrt;
                                      }),
                                    })
                                  }
                                />

                                <Accordion
                                  expanded={expanded_ac === "wp " + index_wp}
                                  onChange={handleChange_ac("wp " + index_wp)}
                                >
                                  <AccordionSummary expandIcon={<ExpandMore />}>
                                    <Typography
                                      sx={{ width: "33%", flexShrink: 0 }}
                                    >
                                      Actions
                                    </Typography>
                                  </AccordionSummary>
                                  <AccordionDetails className={classes.details}>
                                    {waypoint.action &&
                                      React.Children.toArray(
                                        Object.keys(waypoint.action).map(
                                          (action_key, index_ac, list_ac) => (
                                            <Fragment
                                              key={
                                                "fragment-action-" + index_ac
                                              }
                                            >
                                              <div>
                                                <Typography
                                                  variant="subtitle1"
                                                  className={
                                                    classes.attributeName
                                                  }
                                                >
                                                  {action_key}
                                                </Typography>
                                                <div
                                                  className={
                                                    classes.actionValue
                                                  }
                                                >
                                                  <TextField
                                                    required
                                                    fullWidth={true}
                                                    value={
                                                      waypoint.action[
                                                        action_key
                                                      ]
                                                        ? waypoint.action[
                                                            action_key
                                                          ]
                                                        : 0
                                                    }
                                                    onChange={(e) =>
                                                      setmission({
                                                        ...mission,
                                                        route:
                                                          mission.route.map(
                                                            (rt) => {
                                                              let copiedrt =
                                                                JSON.parse(
                                                                  JSON.stringify(
                                                                    rt
                                                                  )
                                                                );
                                                              rt ==
                                                              mission.route[
                                                                index
                                                              ]
                                                                ? (copiedrt.wp[
                                                                    index_wp
                                                                  ]["action"][
                                                                    action_key
                                                                  ] =
                                                                    e.target.value)
                                                                : (copiedrt =
                                                                    rt);
                                                              return copiedrt;
                                                            }
                                                          ),
                                                      })
                                                    }
                                                  />
                                                </div>
                                                <IconButton
                                                  sx={{
                                                    py: 0,
                                                    pr: 2,
                                                    marginLeft: "auto",
                                                  }}
                                                  onClick={() =>
                                                    Removing_action(
                                                      index,
                                                      index_wp,
                                                      action_key
                                                    )
                                                  }
                                                  className={classes.negative}
                                                >
                                                  <DeleteIcon />
                                                </IconButton>
                                              </div>
                                              <Divider></Divider>
                                            </Fragment>
                                          )
                                        )
                                      )}
                                    <Box textAlign="center">
                                      {newactionmenu ? (
                                        <Button
                                          variant="contained"
                                          size="large"
                                          sx={{ width: "80%", flexShrink: 0 }}
                                          style={{ marginTop: "15px" }}
                                          onClick={handleChange_acnew(
                                            "wp " + index_wp
                                          )}
                                        >
                                          Add new action
                                        </Button>
                                      ) : (
                                        <div>
                                          <Typography variant="subtitle1">
                                            Tipo de acccion a añadir
                                          </Typography>
                                          <SelectField
                                            emptyValue={null}
                                            fullWidth={true}
                                            value={newactionid}
                                            onChange={(e) =>
                                              setnewactionid(e.target.value)
                                            }
                                            endpoint={
                                              "/api/mission/actions/dji"
                                            }
                                            keyGetter={(it) => it.id}
                                            titleGetter={(it) => it.description}
                                          />
                                          <div>
                                            <Button
                                              onClick={() =>
                                                setnewactionmenu(true)
                                              }
                                            >
                                              Cancel
                                            </Button>
                                            <Button
                                              onClick={() =>
                                                setnewaction(index, index_wp)
                                              }
                                            >
                                              Add
                                            </Button>
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
                                    sx={{ width: "80%", flexShrink: 0 }}
                                    style={{ marginTop: "15px" }}
                                    onClick={() => AddnewWp(index, index_wp)}
                                  >
                                    Add new Waypoint
                                  </Button>
                                </Box>
                              </AccordionDetails>
                            </Accordion>
                          )
                        )
                      )}

                      <Box textAlign="center">
                        <Button
                          value={index}
                          variant="contained"
                          size="large"
                          sx={{ width: "80%", flexShrink: 0 }}
                          style={{ marginTop: "15px" }}
                          onClick={(e) => AddnewWp(e.target.value, -1)}
                        >
                          Add new Waypoint
                        </Button>
                      </Box>
                    </AccordionDetails>
                  </Accordion>
                  {index < list.length - 1 ? <Divider /> : null}
                </Fragment>
              ))
            )}
            <Box textAlign="center">
              <Button
                variant="contained"
                size="large"
                sx={{ width: "80%", flexShrink: 0 }}
                style={{ marginTop: "15px" }}
                onClick={AddnewRoute}
              >
                Add new Route{" "}
              </Button>
            </Box>
          </Box>
        </Fragment>
      )}
    </Fragment>
  );
};

export default RoutesList;
