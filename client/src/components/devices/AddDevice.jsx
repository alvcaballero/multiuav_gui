import { useState } from 'react';
import { makeStyles } from 'tss-react/mui';

import {
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  IconButton,
  Button,
  Typography,
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';

import DeviceIdentityEditor from './DeviceIdentityEditor';
import { capabilityEnabled, isDeviceIdentityValid } from './deviceIdentity';
import useDeviceModel from './useDeviceModel';
import DeviceCameraEditor from './DeviceCameraEditor';
import DeviceFilesEditor from './DeviceFilesEditor';
import { addDevice } from '../../common/util/fetchs';
import { useCatch } from '../../reactHelper';

const useStyles = makeStyles()((theme) => ({
  title: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    borderBottom: `1px solid ${theme.palette.divider}`,
  },
  body: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1),
    padding: theme.spacing(2),
    background: theme.palette.background.default,
  },
  footer: {
    borderTop: `1px solid ${theme.palette.divider}`,
    padding: theme.spacing(1, 2),
  },
}));

/**
 * Add-device dialog. The same editors as the settings page (identity, camera sources, file
 * sources); it closes only once POST /api/devices accepts the device, so a rejected one (a
 * duplicate name, a system id already in use) keeps what the operator typed.
 */
const AddDevice = ({ onClose }) => {
  const { classes } = useStyles();
  const [item, setItem] = useState({ name: 'uav_', camera: [], files: [] });
  const [saving, setSaving] = useState(false);
  // Its model's capabilities decide which source settings it has (the stream, the files).
  const model = useDeviceModel(item.deviceModel);

  const handleAdd = useCatch(async () => {
    setSaving(true);
    try {
      await addDevice(item);
      onClose();
    } finally {
      setSaving(false);
    }
  });

  const valid = isDeviceIdentityValid(item);

  return (
    <Dialog open onClose={onClose} maxWidth="sm" fullWidth scroll="paper">
      <DialogTitle className={classes.title}>
        Add device
        <IconButton size="small" onClick={onClose} aria-label="Close">
          <CloseIcon fontSize="small" />
        </IconButton>
      </DialogTitle>
      <DialogContent className={classes.body}>
        <DeviceIdentityEditor value={item} onChange={setItem} />
        {capabilityEnabled(item, model, 'camera_stream') && (
          <DeviceCameraEditor
            value={item.camera}
            onChange={(camera) => setItem({ ...item, camera })}
          />
        )}
        {capabilityEnabled(item, model, 'file_download') && (
          <DeviceFilesEditor value={item.files} onChange={(files) => setItem({ ...item, files })} />
        )}
      </DialogContent>
      <DialogActions className={classes.footer}>
        <Typography variant="body2" color="textSecondary" sx={{ mr: 'auto' }}>
          {valid ? 'Ready to add.' : 'Fill in the required fields to add it.'}
        </Typography>
        <Button variant="outlined" onClick={onClose}>
          Cancel
        </Button>
        <Button variant="contained" onClick={handleAdd} disabled={!valid || saving}>
          Add device
        </Button>
      </DialogActions>
    </Dialog>
  );
};

export default AddDevice;
