import React,{ useContext,useState } from 'react'
import { RosContext } from './RosControl'
import useClasses from './useClasses'
import CloseIcon from '@mui/icons-material/Close';

import {
  Card,
  IconButton,
  MenuItem,
  Button,
  Select,
  TextField,
  FormControl,
  InputLabel,
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
    top: '35%',
    transform: 'translateX(-50%)',
  },
  button: {
    width: '80%',
    paddingBottom:'10pt',
  },
  formControl: {
    margin: theme.spacing(1),
    width: '80%',
    minWidth: 120,
    paddingBottom:'20pt',
  },
  inputtext:{
    paddingBottom:'10pt',
  }
});


export const Adduav = ({SetAddUAVOpen}) => {
    const classes = useClasses(styles);
    const rosContex = useContext(RosContext);
    const [uavtype, setuavtype] = useState('dji');
    const [uavid, setuavid] = useState('');

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
        console.log("add uav-"+uavid+"-"+uavtype)
        rosContex.connectAddUav(uavid,uavtype)
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
            <TextField
            required
            label="UAV ID"
            name="uavid"
            value={uavid}
            onChange={(e) => setuavid(e.target.value)} className={classes.inputtext}
          />
      <FormControl variant="outlined" className={classes.formControl}>
        <InputLabel id="demo-simple-select-outlined-label">UAV type</InputLabel>
        <Select
          labelId="demo-simple-select-outlined-label"
          id="demo-simple-select-outlined"
          value={uavtype}
          onChange={(e) => setuavtype(e.target.value)}
          label="uavtipe"
        >
          <MenuItem value="dji">DJI</MenuItem>
          <MenuItem value="px4">PX4</MenuItem>
          <MenuItem value="fuvex">Fuvex</MenuItem>
          <MenuItem value="catec">Catec</MenuItem>
        </Select>
      </FormControl>
  
      <div style={{paddingBottom: '20pt'}}>
        <Button className={classes.button} variant="contained" onClick={AddnewUAV}>ADD</Button>
        </div>                

      </Card>
      </div>
  )
}