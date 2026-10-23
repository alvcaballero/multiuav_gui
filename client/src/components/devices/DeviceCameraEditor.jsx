import { makeStyles } from 'tss-react/mui';
import {
  Button,
  TextField,
  MenuItem,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
} from '@mui/material';
import AddIcon from '@mui/icons-material/Add';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import DeviceSourceCard from './DeviceSourceCard';

const useStyles = makeStyles()((theme) => ({
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1.5),
    paddingBottom: theme.spacing(2),
  },
  fields: {
    display: 'grid',
    gridTemplateColumns: '170px 1fr 1fr',
    gap: theme.spacing(1.5),
    [theme.breakpoints.down('sm')]: {
      gridTemplateColumns: '1fr',
    },
  },
  add: {
    alignSelf: 'flex-start',
  },
}));

// What `host` means for each type, as server/schemas/zod/devices.js (CameraSchema) defines it.
// The server also accepts RTSP, but the client has no player for it, so it is not offered.
const CAMERA_TYPES = {
  WebRTC: {
    label: 'WebRTC (via GCS)',
    help: 'The GCS media server (MediaMTX) pulls the stream from the host and serves it as WebRTC.',
  },
  WebRTC_env: {
    label: 'WebRTC (device server)',
    help: 'The device serves WebRTC itself: the browser connects to the host directly.',
  },
  Websocket: {
    label: 'WebSocket (ROS frames)',
    help: 'Compressed images published by the device arrive through the GCS. No host needed.',
  },
};

const hasHost = (type) => type !== 'Websocket';

// Controlled editor for a device's camera-source array. Owns the accordion,
// add/remove and per-source fields; the parent only supplies value/onChange.
// An empty host is removed, not sent as '': the server only accepts it absent or non-empty.
const DeviceCameraEditor = ({ value, onChange }) => {
  const { classes } = useStyles();
  const cameras = value ?? [];

  const setCamera = (index, patch) => {
    onChange(
      cameras.map((cam, camIndex) => {
        if (camIndex !== index) return cam;
        const next = { ...cam, ...patch };
        if (!hasHost(next.type) || !next.host) delete next.host;
        return next;
      }),
    );
  };

  const addCamera = () => onChange([...cameras, { type: 'WebRTC', source: '' }]);

  const removeCamera = (index) => onChange(cameras.filter((_, camIndex) => camIndex !== index));

  return (
    <Accordion>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
        <Typography variant="subtitle1">
          Camera streams{cameras.length ? ` (${cameras.length})` : ''}
        </Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {cameras.length === 0 && (
          <Typography variant="body2" color="textSecondary">
            No camera streams. Add one to watch it from the GCS.
          </Typography>
        )}
        {/* No stable id on a camera entry yet (see client/CLAUDE.md › Missing stable IDs): the
            fields are fully controlled, so an index key only costs focus on a mid-list delete. */}
        {cameras.map((camera, index) => (
          <DeviceSourceCard
            key={index}
            title={`Camera ${index + 1}`}
            onRemove={() => removeCamera(index)}
          >
            <div className={classes.fields}>
              <TextField
                select
                label="Type"
                value={camera.type}
                onChange={(e) => setCamera(index, { type: e.target.value })}
              >
                {Object.entries(CAMERA_TYPES).map(([type, { label }]) => (
                  <MenuItem key={type} value={type}>
                    {label}
                  </MenuItem>
                ))}
              </TextField>
              <TextField
                required
                label="Source"
                value={camera.source ?? ''}
                onChange={(e) => setCamera(index, { source: e.target.value })}
              />
              {hasHost(camera.type) && (
                <TextField
                  required
                  label="Host"
                  value={camera.host ?? ''}
                  onChange={(e) => setCamera(index, { host: e.target.value.trim() })}
                  error={!camera.host}
                />
              )}
            </div>
            <Typography variant="caption" color="textSecondary">
              {CAMERA_TYPES[camera.type]?.help}
            </Typography>
          </DeviceSourceCard>
        ))}
        <Button
          variant="outlined"
          startIcon={<AddIcon />}
          onClick={addCamera}
          className={classes.add}
        >
          Add camera
        </Button>
      </AccordionDetails>
    </Accordion>
  );
};

export default DeviceCameraEditor;
