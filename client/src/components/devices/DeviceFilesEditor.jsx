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
    gridTemplateColumns: '170px 1fr',
    gap: theme.spacing(1.5),
    [theme.breakpoints.down('sm')]: {
      gridTemplateColumns: '1fr',
    },
  },
  advanced: {
    background: 'transparent',
    '&::before': { display: 'none' },
  },
  advancedSummary: {
    minHeight: 0,
    padding: 0,
    '& .MuiAccordionSummary-content': { margin: theme.spacing(0.5, 0) },
  },
  advancedDetails: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr',
    gap: theme.spacing(1.5),
    padding: theme.spacing(1, 0, 0),
    [theme.breakpoints.down('sm')]: {
      gridTemplateColumns: '1fr',
    },
  },
  fullRow: {
    gridColumn: '1 / -1',
  },
  add: {
    alignSelf: 'flex-start',
  },
}));

// The presets of server/config/devices/devices.yaml (files.<type>).
const FILE_TYPES = {
  onboard_computer: 'Onboard computer',
  wiris_pro: 'Wiris Pro',
  default: 'Default',
};

// Serialize a boolean override to the Select value ('' means "use preset").
const boolToSelect = (v) => (v === undefined ? '' : String(v));
// Parse the Select value back: '' clears the override, otherwise a real boolean.
const selectToBool = (v) => (v === '' ? '' : v === 'true');

const PRESET = (
  <MenuItem value="">
    <em>Use preset</em>
  </MenuItem>
);

// Controlled editor for a device's file-source array. Owns the accordion,
// add/remove, per-source fields and the advanced preset-override block; the
// parent only supplies value/onChange. An empty override field falls back to
// the server preset (devices.yaml) for that source type.
const DeviceFilesEditor = ({ value, onChange }) => {
  const { classes } = useStyles();
  const files = value ?? [];

  // Set/clear a single field on the file entry at `index`. An empty value
  // removes the override so the server preset keeps applying for that field.
  const setFileField = (index, field, fieldValue) => {
    onChange(
      files.map((file, fileIndex) => {
        if (fileIndex !== index) return file;
        const next = { ...file };
        if (fieldValue === '' || fieldValue === undefined) delete next[field];
        else next[field] = fieldValue;
        return next;
      }),
    );
  };

  const addFile = () => onChange([...files, { type: 'onboard_computer', url: '' }]);

  const removeFile = (index) => onChange(files.filter((_, fileIndex) => fileIndex !== index));

  return (
    <Accordion>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
        <Typography variant="subtitle1">
          File sources{files.length ? ` (${files.length})` : ''}
        </Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {files.length === 0 && (
          <Typography variant="body2" color="textSecondary">
            No file sources. Add one to download the media the device records.
          </Typography>
        )}
        {/* Same index-key trade-off as DeviceCameraEditor: no stable id on an entry yet. */}
        {files.map((file, index) => (
          <DeviceSourceCard
            key={index}
            title={`Source ${index + 1}`}
            onRemove={() => removeFile(index)}
          >
            <div className={classes.fields}>
              <TextField
                select
                label="Preset"
                value={file.type}
                onChange={(e) => setFileField(index, 'type', e.target.value)}
              >
                {Object.entries(FILE_TYPES).map(([type, label]) => (
                  <MenuItem key={type} value={type}>
                    {label}
                  </MenuItem>
                ))}
              </TextField>
              <TextField
                required
                label="URL"
                placeholder="sftp://user:password@host:port"
                value={file.url ?? ''}
                onChange={(e) => setFileField(index, 'url', e.target.value)}
                helperText="SFTP or FTP"
              />
            </div>
            <Accordion disableGutters elevation={0} className={classes.advanced}>
              <AccordionSummary
                expandIcon={<ExpandMoreIcon fontSize="small" />}
                className={classes.advancedSummary}
              >
                <Typography variant="body2" color="textSecondary">
                  Override preset
                </Typography>
              </AccordionSummary>
              <AccordionDetails className={classes.advancedDetails}>
                <TextField
                  label="Remote path"
                  placeholder="./uav_media/"
                  value={file.path ?? ''}
                  onChange={(e) => setFileField(index, 'path', e.target.value)}
                  className={classes.fullRow}
                />
                <TextField
                  select
                  label="Download"
                  value={file.downloadType ?? ''}
                  onChange={(e) => setFileField(index, 'downloadType', e.target.value)}
                >
                  {PRESET}
                  <MenuItem value="all">All folders</MenuItem>
                  <MenuItem value="lastFolder">Last folder</MenuItem>
                  <MenuItem value="specific">Specific folder</MenuItem>
                </TextField>
                <TextField
                  select
                  label="Delete after download"
                  value={boolToSelect(file.delete)}
                  onChange={(e) => setFileField(index, 'delete', selectToBool(e.target.value))}
                >
                  {PRESET}
                  <MenuItem value="true">Yes</MenuItem>
                  <MenuItem value="false">No</MenuItem>
                </TextField>
                <TextField
                  select
                  label="Service download"
                  value={boolToSelect(file.srvDownload)}
                  onChange={(e) => setFileField(index, 'srvDownload', selectToBool(e.target.value))}
                >
                  {PRESET}
                  <MenuItem value="true">Yes</MenuItem>
                  <MenuItem value="false">No</MenuItem>
                </TextField>
                <Typography variant="caption" color="textSecondary" className={classes.fullRow}>
                  Leave a field on its preset to keep the value from the server (devices.yaml).
                </Typography>
              </AccordionDetails>
            </Accordion>
          </DeviceSourceCard>
        ))}
        <Button
          variant="outlined"
          startIcon={<AddIcon />}
          onClick={addFile}
          className={classes.add}
        >
          Add file source
        </Button>
      </AccordionDetails>
    </Accordion>
  );
};

export default DeviceFilesEditor;
