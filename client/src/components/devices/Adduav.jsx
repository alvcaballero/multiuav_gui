import { useState } from 'react';
import { makeStyles } from 'tss-react/mui';

import CloseIcon from '@mui/icons-material/Close';
import { Card, IconButton, Button } from '@mui/material';
import DeviceIdentityEditor from './DeviceIdentityEditor';
import { isDeviceIdentityValid } from './deviceIdentity';
import DeviceCameraEditor from './DeviceCameraEditor';
import DeviceFilesEditor from './DeviceFilesEditor';
import { addDevice } from '../../common/util/fetchs';
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

        <DeviceIdentityEditor value={item} onChange={setItem} />
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
          <Button
            className={classes.button}
            variant="contained"
            onClick={addNewUAV}
            disabled={!isDeviceIdentityValid(item)}
          >
            Add new device
          </Button>
        </div>
      </Card>
    </div>
  );
};
export default Adduav;
