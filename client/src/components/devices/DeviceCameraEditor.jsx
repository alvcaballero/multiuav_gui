import { Fragment } from 'react';
import { makeStyles } from 'tss-react/mui';
import {
  Divider,
  IconButton,
  Button,
  Select,
  TextField,
  FormControl,
  InputLabel,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  MenuItem,
} from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';

const useStyles = makeStyles()((theme) => ({
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
  actionValue: {
    display: 'inline-block',
    width: '55%',
  },
}));

// Controlled editor for a device's camera-source array. Owns the accordion,
// add/remove and per-source fields; the parent only supplies value/onChange.
const DeviceCameraEditor = ({ value, onChange }) => {
  const { classes } = useStyles();
  const cameras = value ?? [];

  const setCameraField = (index, field, fieldValue) => {
    onChange(
      cameras.map((cam, camIndex) => {
        if (camIndex !== index) return cam;
        const mycam = structuredClone(cam);
        mycam[field] = fieldValue;
        return mycam;
      }),
    );
  };

  const addCamera = () => {
    onChange([...structuredClone(cameras), { type: 'WebRTC', source: '' }]);
  };

  const removeCamera = (index) => {
    const next = structuredClone(cameras);
    next.splice(index, 1);
    onChange(next);
  };

  return (
    <Accordion>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
        <Typography variant="subtitle1">Camera Stream</Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        <Typography variant="caption">Source example:main</Typography>
        {cameras.map((camera, index) => (
          <Fragment key={'fragment-action-' + index}>
            <Typography variant="subtitle1" className={classes.attributeName}>
              {'Camera ' + index}
            </Typography>
            <div>
              <FormControl variant="outlined">
                <InputLabel id={'camera-type-label-' + index}>CameraType</InputLabel>
                <Select
                  labelId={'camera-type-label-' + index}
                  value={camera['type']}
                  label="type"
                  onChange={(e) => setCameraField(index, 'type', e.target.value)}
                >
                  <MenuItem value="WebRTC">WebRTC</MenuItem>
                  <MenuItem value="WebRTC_env">WebRTCenv</MenuItem>
                  <MenuItem value="Websocket">Websocket</MenuItem>
                </Select>
              </FormControl>
              <div className={classes.actionValue}>
                <TextField
                  required
                  fullWidth={true}
                  label="Source"
                  value={camera['source']}
                  onChange={(e) => setCameraField(index, 'source', e.target.value)}
                />
              </div>
              <IconButton
                sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                onClick={() => removeCamera(index)}
              >
                <DeleteIcon />
              </IconButton>
            </div>
            <Divider />
          </Fragment>
        ))}

        <Button variant="contained" onClick={addCamera}>
          Add camera source
        </Button>
      </AccordionDetails>
    </Accordion>
  );
};

export default DeviceCameraEditor;
