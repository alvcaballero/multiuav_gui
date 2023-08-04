import React, { Fragment, useEffect,useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import makeStyles from '@mui/styles/makeStyles';
import {
  Divider, 
  List, 
  ListItemButton, 
  ListItemText,
  Collapse,
  IconButton,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
} from '@mui/material';
import ArrowRight from '@mui/icons-material/ArrowRight';
import KeyboardArrowDown from '@mui/icons-material/KeyboardArrowDown';
import Typography from '@mui/material/Typography';
import ExpandLess from '@mui/icons-material/ExpandLess';
import ExpandMore from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import DeleteIcon from '@mui/icons-material/Delete';
import AddIcon from '@mui/icons-material/Add';

//import { geofencesActions } from '../store';
//import CollectionActions from '../settings/components/CollectionActions';
//import { useCatchCallback } from '../reactHelper';
// https://dev.to/shareef/how-to-work-with-arrays-in-reactjs-usestate-4cmi

const useStyles = makeStyles(() => ({
    list: {
      maxHeight: '100%',
      overflow: 'auto',
    },
    icon: {
      width: '25px',
      height: '25px',
      filter: 'brightness(0) invert(1)',
    },
  }));

const RoutesList = ({ onGeofenceSelected }) => {

    const classes = useStyles();
    const Mission_route = useSelector((state) => state.mission);

    const [items, setItems] = useState({name:"no mission"});
    const [open_routes, setOpen_routes] = useState(true);
    const [expanded_route, setExpanded_route] = useState(false);
    const [expanded_wp, setExpanded_wp] = useState(false);
    const [expanded_ac, setExpanded_ac] = useState(false);
    const [add_mission, setaddMission] = useState(true)

    useEffect(()=>{
      setItems(Mission_route)
      console.log("edit mission");
      console.log(Mission_route);
      Mission_route.route.length >0 ? setaddMission(false) : setaddMission(true);
    },[Mission_route])
    const changetype = (value) =>{
      console.log(value)
    }
    const Removing_wp = () =>{

    }

    const handleChange_route = (panel) => (event, isExpanded) => {
      setExpanded_route(isExpanded ? panel : false);
    };
    const handleChange_wp = (panel) => (event, isExpanded) => {
      setExpanded_wp(isExpanded ? panel : false);
    };
    const handleChange_ac = (panel) => (event, isExpanded) => {
      setExpanded_ac(isExpanded ? panel : false);
    };
  return (
    <div>
    {add_mission ? 
        <IconButton onClick={() => console.log("save")} >
              <AddIcon />
        </IconButton>
    :
    <Fragment>
      <div >
        <Typography variant="subtitle1" style={{display: 'inline'}} >Name Mission</Typography>
        <TextField
              required
              name="Name Mission"
              style={{display: 'inline',width: "200px"}}
              value={items.name}
              onChange={(event) => setItems({ ...items, name: event.target.value })}
            />
      </div>
      <div >
        <Typography variant="subtitle1" style={{display: 'inline'}} >Description</Typography>
        <TextField
              required
              name="Name Mission"
              style={{display: 'inline',width: "200px"}}
              value={items.description}
              onChange={(event) => setItems({ ...items, description: event.target.value })}
            />
      </div>
      </Fragment>
    }

    {items.route && 
    React.Children.toArray(
    Object.values(items.route).map((item_route, index, list) => (
      <Fragment key={"fragment-"+item_route.id}>
      <Accordion expanded={expanded_route === ('Rute '+ index)} onChange={handleChange_route('Rute '+ index)} >
        <AccordionSummary expandIcon={<ExpandMore />} >
          <Typography sx={{ width: '33%', flexShrink: 0 }}> {'Rute '+ index}</Typography>
          <Typography sx={{ color: 'text.secondary' }}> {item_route.name+"- "+item_route.uav} </Typography>
        </AccordionSummary>
        <AccordionDetails>
        <div>
            <Typography variant="subtitle1" style={{display: 'inline'}} >Route Name</Typography>
            <TextField
                required
                name="Name Mission"
                value={item_route.name ? item_route.name  : "route name"}
                onChange={(e) => setItems({ ...items, route: items.route.map((rt)=> rt == items.route[index] ? {...item_route, name: e.target.value}:rt)})}
              />
          </div>
        <div >
            <Typography variant="subtitle1" style={{display: 'inline'}} >UAV</Typography>
            <TextField
                required
                name="Name Mission"
                value={item_route.uav ? item_route.uav  : "uav_1"}
                onChange={(e) => setItems({ ...items, route: items.route.map((rt)=> rt == items.route[index] ? {...item_route, uav: e.target.value}:rt)})}
              />
          </div>
        <div >
          <Typography variant="subtitle1" style={{display: 'inline'}} >type</Typography>
          <SelectField
            emptyValue={null}
            value={item_route.uav_type ? item_route.uav_type : ""}
            onChange={(e) => setItems({ ...items, route: items.route.map((rt)=> rt == items.route[index] ? {...item_route, uav_type: e.target.value}:rt)})}
            endpoint="/api/devices/type"
            keyGetter={(it) => it}
            titleGetter={(it) => it}
            label={'Type-uav-mission'}
            style={{display: 'inline',width: "200px"}}
          />
        </div>

        <Accordion expanded={expanded_wp === ('wp '+ "-1")} onChange={handleChange_wp('wp '+ "-1")}>
          <AccordionSummary expandIcon={<ExpandMore />} >
            <Typography sx={{ width: '53%', flexShrink: 0 }}> Route Attributes</Typography>
          </AccordionSummary>
          <AccordionDetails>
          { item_route.attributes &&
            <Fragment>
              <div >
                <Typography variant="subtitle1" style={{display: 'inline'}} >Speed crucer</Typography>
                <TextField
                    required
                    name="speed"
                    style={{display: 'inline',width: "200px"}}
                    value={item_route.attributes.idle_vel ? item_route.attributes.idle_vel  : 1.85}
                    onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == items.route[index] ? copiedrt.attributes.idle_vel = e.target.value : copiedrt=rt;
                      return copiedrt
                    })})}
                  />
              </div>
              <div >
                <Typography variant="subtitle1" style={{display: 'inline'}} > landing mode </Typography>
                <SelectField
                  emptyValue={null}
                  value={item_route.attributes.mode_landing ? item_route.attributes.mode_landing : 0}
                  onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                    let copiedrt = JSON.parse(JSON.stringify(rt));
                    rt === items.route[index] ? copiedrt.attributes.mode_landing = e.target.value : copiedrt=rt;
                    return copiedrt
                    })})}
                  endpoint={"/api/mission/atributesparam/dji/mode_landing"}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                  style={{display: 'inline',width: "200px"}}
                />
              </div>
              <div >
                <Typography variant="subtitle1" style={{display: 'inline'}} > YAW mode </Typography>
                <SelectField
                  emptyValue={null}
                  value={item_route.attributes.mode_yaw ? item_route.attributes.mode_yaw : 0}
                  onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                    let copiedrt = JSON.parse(JSON.stringify(rt));
                    rt === items.route[index] ? copiedrt.attributes.mode_yaw = e.target.value : copiedrt=rt;
                    return copiedrt
                    })})}
                  endpoint={"/api/mission/atributesparam/dji/mode_yaw"}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                  style={{display: 'inline',width: "200px"}}
                />
              </div>
              <div >
                <Typography variant="subtitle1" style={{display: 'inline'}} > Gimbal mode </Typography>
                <SelectField
                  emptyValue={null}
                  value={item_route.attributes.mode_gimbal ? item_route.attributes.mode_gimbal : 0}
                  onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                    let copiedrt = JSON.parse(JSON.stringify(rt));
                    rt === items.route[index] ? copiedrt.attributes.mode_gimbal = e.target.value : copiedrt=rt;
                    return copiedrt
                    })})}
                  endpoint={"/api/mission/atributesparam/dji/mode_gimbal"}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                  style={{display: 'inline',width: "200px"}}
                />
              </div>
              <div >
                <Typography variant="subtitle1" style={{display: 'inline'}} > Trace mode </Typography>
                <SelectField
                  emptyValue={null}
                  value={item_route.attributes.mode_trace ? item_route.attributes.mode_trace : 0}
                  onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                    let copiedrt = JSON.parse(JSON.stringify(rt));
                    rt === items.route[index] ? copiedrt.attributes.mode_trace = e.target.value : copiedrt=rt;
                    return copiedrt
                    })})}
                  endpoint={"/api/mission/atributesparam/dji/mode_trace"}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                  style={{display: 'inline',width: "200px"}}
                />
              </div>
            </Fragment>}
          </AccordionDetails>
        </Accordion>

        <Typography variant="subtitle1">Waypoints</Typography>
        {React.Children.toArray(Object.values(item_route.wp).map((waypoint, index_wp, list_wp) => (
              <Accordion expanded={expanded_wp === ('wp '+ index_wp)} onChange={handleChange_wp('wp '+ index_wp)}>
              <AccordionSummary expandIcon={<ExpandMore />} >
                <Typography sx={{ width: '33%', flexShrink: 0 }}> {'wp '+ index_wp}</Typography>
              </AccordionSummary>
              <AccordionDetails>
                <div>
                <Typography variant="subtitle1" style={{display: 'inline'}} >Position</Typography>
                <IconButton
                  onClick={() => Removing_wp(true)}
                  //disabled={disableActions || deviceReadonly}
                  className={classes.negative}
                >
                  <DeleteIcon />
                </IconButton>
                </div>

                <div >
                  
                  <TextField
                      required
                      label="Latitud "
                      name="Latitud"
                      inputProps={{style: { width: "140px", },}}
                      style={{display: 'inline'}}
                      value={waypoint.pos ? waypoint.pos[0]  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["pos"][0] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                  <TextField
                      required
                      label="Longitud "
                      name="Longitud"
                      inputProps={{style: { width: "140px", },}}
                      style={{display: 'inline'}}
                      value={waypoint.pos ? waypoint.pos[1]  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["pos"][1] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                  <TextField
                      required
                      label="altura "
                      name="Altura"
                      inputProps={{style: { width: "50px", },}}
                      style={{display: 'inline'}}
                      value={waypoint.pos ? waypoint.pos[2]  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["pos"][2] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                </div>
                <div >
                  <Typography variant="subtitle1" style={{display: 'inline'}} >Yaw</Typography>
                  <TextField
                      required
                      label="YAW "
                      name="YAW"
                      style={{display: 'inline',width: "200px"}}
                      value={waypoint.yaw ? waypoint.yaw  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["yaw"] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                </div>
                <div >
                  <Typography variant="subtitle1" style={{display: 'inline'}} >Gimbal</Typography>
                  <TextField
                      required
                      label="Gimbal "
                      name="Gimbal"
                      style={{display: 'inline',width: "200px"}}
                      value={waypoint.gimbal ? waypoint.gimbal  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["gimbal"] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                </div>

              <Accordion expanded={expanded_ac === ('wp '+ index_wp)} onChange={handleChange_ac('wp '+ index_wp)}>
              <AccordionSummary expandIcon={<ExpandMore />} >
                <Typography sx={{ width: '33%', flexShrink: 0 }}> Actions</Typography>
              </AccordionSummary>
              <AccordionDetails>
              {waypoint.action && React.Children.toArray(Object.keys(waypoint.action).map((action_key, index_ac, list_ac) => (
              <div >
                <Typography variant="subtitle1" style={{display: 'inline'}} >{action_key}</Typography>
                <TextField
                    required
                    label={action_key}
                    name={action_key}
                    style={{display: 'inline',width: "200px"}}
                    value={waypoint.action[action_key] ? waypoint.action[action_key]  : 0}
                    onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == items.route[index] ? copiedrt.wp[index_wp]["action"][action_key] = e.target.value : copiedrt=rt;
                      return copiedrt
                    })})}
                  />
                <IconButton
                  onClick={() => Removing_wp(true)}
                  //disabled={disableActions || deviceReadonly}
                  className={classes.negative}
                >
                <DeleteIcon />
                </IconButton>
              </div>
              )))}
              <div>
              <IconButton
                  onClick={() => Removing_wp(true)}
                  //disabled={disableActions || deviceReadonly}
                  className={classes.negative}
                >
                  <DeleteIcon />
                </IconButton>
              </div>

              </AccordionDetails>
            </Accordion>

              </AccordionDetails>
            </Accordion>
        )))}

        </AccordionDetails>
      </Accordion>
        {index < list.length - 1 ? <Divider /> : null}
      </Fragment>
    )))}

  </div>
  )
}

export default RoutesList