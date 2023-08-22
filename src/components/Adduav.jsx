import React,{ useContext,useState } from 'react'
import { RosContext } from './RosControl'
import useClasses from './useClasses'
import CloseIcon from '@mui/icons-material/Close';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Card,
  IconButton,
  MenuItem,
  Button,
  Select,
  TextField,
  FormControl,
  InputLabel,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  FormGroup,
  FormControlLabel,
  Checkbox,
} from '@mui/material';

const styles = theme => ({
  card: {
    pointerEvents: 'auto',
    width: theme.dimensions.popupMaxWidth,
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: "100%",
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
  },
  root: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '50%',
    top: '20%',
    transform: 'translateX(-50%)',
  },
  button: {
    width: '80%',
    paddingBottom:'10pt',
    paddingTop:'10pt',
  },
  formControl: {
    margin: theme.spacing(1),
    gap: theme.spacing(2),
    paddingBottom:'20pt',
  },
  inputtext:{
    paddingBottom:'20px',
  },
    details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
});


export const Adduav = ({SetAddUAVOpen}) => {
    const classes = useClasses(styles);
    const rosContex = useContext(RosContext);
    const [uavtype, setuavtype] = useState("dji");
    const [uavid, setuavid] = useState('');
    const [item, setItem] = useState({name:'uav_',category:'dji',ip:'10.42.0.42',cameratype:'Websocket',camera_src:'video0'});

    const validate = () => item && item.name && item.uniqueId;

	function showAddUav(){
		if(document.getElementById("AddUav").style.width === "250px"){
			document.getElementById("AddUav").style.width = "0";
		}else{
			document.getElementById("AddUav").style.width = "250px";
		}
	}

    function closeAddUav(){
        SetAddUAVOpen(false);

    }
    function AddnewUAV(){
        console.log("add uav-"+item.name+"-"+item.category)
        console.log(item)
        rosContex.connectAddUav(item)
        SetAddUAVOpen(false);

    }
    function disConnectUav (){
      SetAddUAVOpen(false);
    }


  return (
    <div className={classes.root}>
    <Card elevation={3} className={classes.card}>
      <div style={{display:'flex',right:"5px",height:"35px",position:"absolute"}}>
        <IconButton size="small" onClick={closeAddUav} onTouchStart={closeAddUav}>
          <CloseIcon fontSize="small" className={classes.mediaButton} />
        </IconButton>
      </div>
    
      <b><div style={{display: 'block',width:"calc( 100% - 60pt )",paddingLeft:"15pt",paddingTop:"10pt",paddingBottom:"20pt",textAlign:"left"}}>Add device  </div></b>
        
      <FormGroup style={{margin:"20px"}}>
          <TextField
            required
            label="UAV ID"
            name="uavid"
            value={item.name}
            onChange={(event) => setItem({ ...item, name: event.target.value })}
            className={classes.inputtext}
          />

        <FormControl style={{paddingBottom:"20px"}} variant="outlined" >
          <InputLabel id="demo-simple-select-outlined-label">UAV type</InputLabel>
          <Select
            labelId="demo-simple-select-outlined-label"
            id="demo-simple-select-outlined"
            value={item.category}
            onChange={(event) => setItem({ ...item, category: event.target.value })}
            label="uavtipe"
          >
            <MenuItem value="dji">DJI</MenuItem>
            <MenuItem value="px4">PX4</MenuItem>
            <MenuItem value="fuvex">Fuvex</MenuItem>
            <MenuItem value="catec">Catec</MenuItem>
          </Select>
        </FormControl>
        <TextField 
          value={item.ip}
          onChange={(event) => setItem({ ...item, ip: event.target.value })}
          label='ip'
        />
      </FormGroup>
      {item &&  <Accordion >
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Camera
              </Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
            <FormControl variant="outlined" >
              <InputLabel id="demo-simple-select-outlined-label">CameraSource</InputLabel>
              <Select
                labelId="demo-simple-select-outlined-label"
                id="demo-simple-select-outlined"
                value={item.cameratype}
                onChange={(event) => setItem({ ...item, cameratype: event.target.value })}
                label="uavtipe"
              >
                <MenuItem value="Websocket">Websocket</MenuItem>
                <MenuItem value="WebRTC">WebRTC</MenuItem>
              </Select>
            </FormControl>
              <TextField
                value={item.camera_src}
                onChange={(event) => setItem({ ...item, camera_src: event.target.value })}
                label='Camera route'
                helperText='videocompress or video0  '
              />
            </AccordionDetails>
          </Accordion>}
  
      <div style={{paddingBottom: '20pt',paddingTop: '20pt',display: "flex",justifyContent:"center",}}>
        <Button className={classes.button} variant="contained" onClick={AddnewUAV}>ADD</Button>
        </div>                

      </Card>
      </div>
  )
}