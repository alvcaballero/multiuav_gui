import React, { Fragment, useEffect,useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import makeStyles from '@mui/styles/makeStyles';
import {
  Divider, 
  List, 
  ListItemButton, 
  ListItemText,
  Collapse,
  Box,
  Button,
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
import { Padding, Tune } from '@mui/icons-material';

//import { geofencesActions } from '../store';
//import CollectionActions from '../settings/components/CollectionActions';
//import { useCatchCallback } from '../reactHelper';
// https://dev.to/shareef/how-to-work-with-arrays-in-reactjs-usestate-4cmi

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
    attributeName:{
      display:"inline-block",
      width:"40%",
      textAlign:"left",
      verticalAlign:"middle"
    },
    attributeValue:{
      display:"inline-block",
      width:"58%"
    },
    actionValue:{
      display:"inline-block",
      width:"40%"
    }
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
    },[])

    const AddnewMission = () =>{
      setItems({name:"new Mission",description:"",route:[]})
      setaddMission(false)
      console.log("add new mission")
    }
    const AddnewRoute = () =>{
      let auxroute = items.route;
      auxroute.push({name:"",uav:'',attributes:{},wp:[]});
      setItems({...items,route:auxroute})
      console.log("add new route")
    }
    const AddnewWp = (index_route) =>{
      console.log("add new wp"+index_route)
      let auxroute = items.route;
      auxroute[index_route].wp.push({pos:[]});
      setItems({...items,route:auxroute})
    }
    const Removing_wp = () =>{
      console.log("remove uno")
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
    <Fragment>
    {add_mission ? 
      <Box textAlign='center'>
        <Button variant="contained" size='large'  sx={{width: '80%', flexShrink: 0 }} style={{marginTop: '15px'}} onClick={AddnewMission} >Create New Mission </Button>
        </Box>
        :
    <Fragment>
      <Box style={{display: 'flex',gap:'10px',flexDirection: 'column', margin:"20px"}}>
      <TextField
        required
        label = "Name Mission"
        variant='standard'
        value={items.name}
        onChange={(event) => setItems({ ...items, name: event.target.value })}
      />
      <TextField
        required
        label = "Description of mission"
        variant='standard'
        value={items.description}
        onChange={(event) => setItems({ ...items, description: event.target.value })}
      />
      {React.Children.toArray(
      Object.values(items.route).map((item_route, index, list) => (
        <Fragment key={"fragment-"+item_route.id}>
        <Accordion expanded={expanded_route === ('Rute '+ index)} onChange={handleChange_route('Rute '+ index)} >
          <AccordionSummary expandIcon={<ExpandMore />} >
            <Typography sx={{ width: '33%', flexShrink: 0 }}> {'Rute '+ index}</Typography>
            <Typography sx={{ color: 'text.secondary' }}> {item_route.name+"- "+item_route.uav} </Typography>
            <IconButton sx={{ py:0,pr:2,marginLeft: "auto" }} onClick={() => Removing_wp(true)} >
              <DeleteIcon />
            </IconButton>
          </AccordionSummary>
          <AccordionDetails className={classes.details}>

          <TextField
            required
            label="Route Name"
            variant='standard'
            value={item_route.name ? item_route.name  : ""}
            onChange={(e) => setItems({ ...items, route: items.route.map((rt)=> rt == items.route[index] ? {...item_route, name: e.target.value}:rt)})}
          />

          <TextField
            required
            label="UAV id"
            variant='standard'
            value={item_route.uav ? item_route.uav  : "uav_1"}
            onChange={(e) => setItems({ ...items, route: items.route.map((rt)=> rt == items.route[index] ? {...item_route, uav: e.target.value}:rt)})}
          />

          <SelectField
            emptyValue={null}
            value={item_route.uav_type ? item_route.uav_type : ""}
            onChange={(e) => setItems({ ...items, route: items.route.map((rt)=> rt == items.route[index] ? {...item_route, uav_type: e.target.value}:rt)})}
            endpoint="/api/devices/type"
            keyGetter={(it) => it}
            titleGetter={(it) => it}
            label={'Type UAV mission'}
            style={{display: 'inline',width: "200px"}}
          />


          <Accordion expanded={expanded_wp === ('wp '+ "-1")} onChange={handleChange_wp('wp '+ "-1")}>
            <AccordionSummary expandIcon={<ExpandMore />} >
              <Typography sx={{ width: '53%', flexShrink: 0 }}> Route Attributes</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
            { item_route.attributes &&
              <Fragment>
                <div>
                  <Typography variant="subtitle1" className={classes.attributeName} >Speed idle: </Typography>
                  <TextField
                      fullWidth
                      required type="number" className={classes.attributeValue}
                      value={item_route.attributes.idle_vel ? item_route.attributes.idle_vel  : 1.85}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.attributes.idle_vel = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                </div>
                <div >
                  <Typography variant="subtitle1" className={classes.attributeName} > landing mode </Typography>
                  <div className={classes.attributeValue}>
                  <SelectField
                    emptyValue={null} fullWidth={true}
                    value={item_route.attributes.mode_landing ? item_route.attributes.mode_landing : 0}
                    onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt === items.route[index] ? copiedrt.attributes.mode_landing = e.target.value : copiedrt=rt;
                      return copiedrt
                      })})}
                    endpoint={"/api/mission/atributesparam/dji/mode_landing"}
                    keyGetter={(it) => it.id}
                    titleGetter={(it) => it.name}
                  />
                  </div>
                </div>
                <div >
                  <Typography variant="subtitle1" className={classes.attributeName} > YAW mode </Typography>
                  <div className={classes.attributeValue}>
                  <SelectField
                    emptyValue={null} fullWidth={true}
                    value={item_route.attributes.mode_yaw ? item_route.attributes.mode_yaw : 0}
                    onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt === items.route[index] ? copiedrt.attributes.mode_yaw = e.target.value : copiedrt=rt;
                      return copiedrt
                      })})}
                    endpoint={"/api/mission/atributesparam/dji/mode_yaw"}
                    keyGetter={(it) => it.id}
                    titleGetter={(it) => it.name}
                  />
                  </div>
                </div>
                <div >
                  <Typography variant="subtitle1" className={classes.attributeName} > Gimbal mode </Typography>
                  <div className={classes.attributeValue}>
                  <SelectField
                    emptyValue={null} fullWidth={true}
                    value={item_route.attributes.mode_gimbal ? item_route.attributes.mode_gimbal : 0}
                    onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt === items.route[index] ? copiedrt.attributes.mode_gimbal = e.target.value : copiedrt=rt;
                      return copiedrt
                      })})}
                    endpoint={"/api/mission/atributesparam/dji/mode_gimbal"}
                    keyGetter={(it) => it.id}
                    titleGetter={(it) => it.name}
                  />
                  </div>
                </div>
                <div >
                  <Typography variant="subtitle1" className={classes.attributeName} > Trace mode: </Typography>
                  <div className={classes.attributeValue}>
                  <SelectField
                    emptyValue={null} fullWidth={true}
                    value={item_route.attributes.mode_trace ? item_route.attributes.mode_trace : 0}
                    onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt === items.route[index] ? copiedrt.attributes.mode_trace = e.target.value : copiedrt=rt;
                      return copiedrt
                      })})}
                    endpoint={"/api/mission/atributesparam/dji/mode_trace"}
                    keyGetter={(it) => it.id}
                    titleGetter={(it) => it.name}
                  />
                  </div>
                </div>
              </Fragment>}
            </AccordionDetails>
          </Accordion>

          <Typography variant="subtitle1">Waypoints</Typography>
          {React.Children.toArray(Object.values(item_route.wp).map((waypoint, index_wp, list_wp) => (
                <Accordion expanded={expanded_wp === ('wp '+ index_wp)} onChange={handleChange_wp('wp '+ index_wp)}>
                <AccordionSummary expandIcon={<ExpandMore />} >
                  <Typography sx={{ width: '33%', flexShrink: 0 }}> {'WP '+ index_wp}</Typography>
                  <IconButton sx={{ py:0,pr:2,marginLeft: "auto" }} onClick={() => Removing_wp(true)} >
                          <DeleteIcon />
                  </IconButton>
                </AccordionSummary>
                <AccordionDetails className={classes.details}>


                    <Box component="form"
                      sx={{
                        '& .MuiTextField-root': { m: 1 },
                      }} >
                        <div>
                        <Typography variant="subtitle1" style={{display: 'inline'}} >Position</Typography>
                        </div>
                        <TextField
                        required
                        label="Latitud "
                        type="number"
                        sx={{ width: '15ch'}}
                        variant='standard'
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
                        type="number"
                        variant='standard'
                        sx={{ width: '15ch'}}
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
                      type="number"
                      variant='standard'
                      sx={{ width: '7ch'}}
                      value={waypoint.pos ? waypoint.pos[2]  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["pos"][2] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                  </Box>

                  <TextField
                      required
                      label="YAW "
                      type="number"
                      variant='standard'
                      value={waypoint.yaw ? waypoint.yaw  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["yaw"] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />


                  <TextField
                      required
                      label="Gimbal "
                      type="number"
                      variant='standard'
                      value={waypoint.gimbal ? waypoint.gimbal  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["gimbal"] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />


                <Accordion expanded={expanded_ac === ('wp '+ index_wp)} onChange={handleChange_ac('wp '+ index_wp)}>
                <AccordionSummary expandIcon={<ExpandMore />} >
                  <Typography sx={{ width: '33%', flexShrink: 0 }}> Actions</Typography>
                </AccordionSummary>
                <AccordionDetails className={classes.details}>
                {waypoint.action && React.Children.toArray(Object.keys(waypoint.action).map((action_key, index_ac, list_ac) => (
                <Fragment>
                <div >
                  <Typography variant="subtitle1" className={classes.attributeName} >{action_key}</Typography>
                  <div className={classes.actionValue}>
                  <TextField
                      required  fullWidth={true}
                      value={waypoint.action[action_key] ? waypoint.action[action_key]  : 0}
                      onChange={(e) => setItems({ ...items, route: items.route.map((rt)=>{
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt == items.route[index] ? copiedrt.wp[index_wp]["action"][action_key] = e.target.value : copiedrt=rt;
                        return copiedrt
                      })})}
                    />
                  </div>
                  <IconButton sx={{ py:0,pr:2,marginLeft: "auto" }} onClick={() => Removing_wp(true)}  className={classes.negative}  >
                    <DeleteIcon />
                  </IconButton>
                </div>
                <Divider></Divider>
                </Fragment>
                )))}
                <Box textAlign='center'>
                  <Button variant="contained" size='large'  sx={{width: '80%', flexShrink: 0 }} style={{marginTop: '15px'}} onClick={() => console.log("save")}>Add new action </Button>
                </Box>

                </AccordionDetails>
              </Accordion>

                </AccordionDetails>
              </Accordion>
          )))}

          <Box textAlign='center'>
            <Button value={index} variant="contained" size='large'  sx={{width: '80%', flexShrink: 0 }} style={{marginTop: '15px'}} onClick={e => AddnewWp(e.target.value)}>Add new Waypoint </Button>
          </Box>

          </AccordionDetails>
        </Accordion>
          {index < list.length - 1 ? <Divider /> : null}
        </Fragment>
      )))}
      <Box textAlign='center'>
        <Button variant="contained" size='large'  sx={{width: '80%', flexShrink: 0 }} style={{marginTop: '15px'}} onClick={AddnewRoute}>Add new Route </Button>
      </Box>
      </Box>
    </Fragment>
    }
  </Fragment>
  )
}

export default RoutesList