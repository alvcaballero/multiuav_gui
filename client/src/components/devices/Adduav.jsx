import { useState } from 'react';
import { makeStyles } from 'tss-react/mui';

import CloseIcon from '@mui/icons-material/Close';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Card,
  IconButton,
  Button,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
} from '@mui/material';
import SelectField from '../../shared/components/SelectField';
import DeviceCameraEditor from './DeviceCameraEditor';
import DeviceFilesEditor from './DeviceFilesEditor';
import { addDevice } from '../../shared/fetchs';
import { useCatch } from '../../reactHelper';
const useStyles = makeStyles()((theme) => ({
  root: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '50%',
    top: '10%',
    transform: 'translateX(-50%)',
  },
  card: {
    pointerEvents: 'auto',
    display: 'block',
    width: '600px',
    height: '80vh',
    transitionDuration: '0.3s',
    overflowY: 'auto',
    padding: theme.spacing(1),
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

  button: {
    width: '80%',
    paddingBottom: '10pt',
    paddingTop: '10pt',
  },
  formControl: {
    margin: theme.spacing(1),
    gap: theme.spacing(2),
    paddingBottom: '20pt',
  },
  inputtext: {
    paddingBottom: '20px',
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: '10pt',
    paddingBottom: '20pt',
  },
  title: {
    display: 'block',
    width: 'calc( 100% - 60pt )',
    paddingLeft: '15pt',
    paddingTop: '5pt',
    paddingBottom: '5pt',
    textAlign: 'center',
  },
  closeBtn: {
    display: 'flex',
    right: '5px',
    height: '35px',
    position: 'absolute',
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
    width: '55%',
  },
}));

const Adduav = ({ SetAddUAVOpen }) => {
  const { classes } = useStyles();
  const handleAddDevice = useCatch(addDevice);
  const [item, setItem] = useState({
    name: 'uav_',
    ip: '10.42.0.42',
    camera: [],
    files: [],
  });

  function closeAddUav() {
    SetAddUAVOpen(false);
  }
  function addNewUAV() {
    handleAddDevice(item);
    SetAddUAVOpen(false);
  }
  return (
    <div className={classes.root}>
      <Card elevation={3} className={classes.card}>
        <div className={classes.closeBtn}>
          <IconButton size="small" onClick={closeAddUav} onTouchStart={closeAddUav}>
            <CloseIcon fontSize="small" className={classes.mediaButton} />
          </IconButton>
        </div>

        <b>
          <div className={classes.title}>Add device </div>
        </b>

        <Accordion defaultExpanded>
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Typography variant="subtitle1">Obligatory data</Typography>
          </AccordionSummary>
          <AccordionDetails className={classes.details}>
            <TextField
              required
              label="Name"
              name="uavid"
              value={item.name}
              onChange={(event) => setItem({ ...item, name: event.target.value })}
              helperText="The name must be unique, it will be used to identify the device, must to be the same of that device's name_space or device's identificator ,it is recommended to use the format uav_XXXX in case of using a UAV"
            />
            <SelectField
              emptyValue={null}
              value={item.category ? item.category : null}
              onChange={(e) => setItem({ ...item, category: e.target.value })}
              endpoint="/api/category"
              keyGetter={(it) => it}
              titleGetter={(it) => it}
              label={'Type '}
            />
            {item.category && (
              <SelectField
                emptyValue={null}
                value={item.protocol ? item.protocol : null}
                onChange={(e) => setItem({ ...item, protocol: e.target.value })}
                endpoint="/api/server/protocol"
                keyGetter={(it) => it}
                titleGetter={(it) => it}
                label={'Protocol '}
              />
            )}
            <TextField
              label="ip"
              value={item.ip}
              onChange={(event) => setItem({ ...item, ip: event.target.value })}
              helperText="IP for camera stream"
            />
          </AccordionDetails>
        </Accordion>
        {item && (
          <>
            <DeviceCameraEditor
              value={item.camera}
              onChange={(camera) => setItem({ ...item, camera })}
            />
            <DeviceFilesEditor
              value={item.files}
              onChange={(files) => setItem({ ...item, files })}
            />
          </>
        )}

        <div
          style={{
            paddingBottom: '20pt',
            paddingTop: '20pt',
            display: 'flex',
            justifyContent: 'center',
          }}
        >
          <Button className={classes.button} variant="contained" onClick={addNewUAV}>
            Add new device
          </Button>
        </div>
      </Card>
    </div>
  );
};
export default Adduav;
