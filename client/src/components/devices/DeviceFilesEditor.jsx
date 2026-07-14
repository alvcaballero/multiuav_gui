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

// Serialize a boolean override to the Select value ('' means "use preset").
const boolToSelect = (v) => (v === undefined ? '' : String(v));
// Parse the Select value back: '' clears the override, otherwise a real boolean.
const selectToBool = (v) => (v === '' ? '' : v === 'true');

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
        const myfile = structuredClone(file);
        if (fieldValue === '' || fieldValue === undefined) delete myfile[field];
        else myfile[field] = fieldValue;
        return myfile;
      }),
    );
  };

  const addFile = () => {
    onChange([...structuredClone(files), { type: 'onboard_computer', url: '' }]);
  };

  const removeFile = (index) => {
    const next = structuredClone(files);
    next.splice(index, 1);
    onChange(next);
  };

  return (
    <Accordion>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
        <Typography variant="subtitle1">Device Files Resurces</Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        <Typography variant="caption">
          The url format sftp://user:password@Ip:port It can be compatible for sftp or ftp protovol
        </Typography>
        {files.map((file, index) => (
          <Fragment key={'fragment-action-file' + index}>
            <Typography variant="subtitle1" className={classes.attributeName}>
              {'File ' + index}
            </Typography>
            <div>
              <FormControl variant="outlined">
                <InputLabel id={'file-type-label-' + index}>Source type</InputLabel>
                <Select
                  labelId={'file-type-label-' + index}
                  value={file['type']}
                  label="type"
                  onChange={(e) => setFileField(index, 'type', e.target.value)}
                >
                  <MenuItem value="onboard_computer">Onboard computer</MenuItem>
                  <MenuItem value="wiris_pro">Wiris_pro</MenuItem>
                  <MenuItem value="default">default</MenuItem>
                </Select>
              </FormControl>
              <div className={classes.actionValue}>
                <TextField
                  required
                  fullWidth={true}
                  label="URL"
                  value={file['url']}
                  onChange={(e) => setFileField(index, 'url', e.target.value)}
                />
              </div>
              <IconButton
                sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                onClick={() => removeFile(index)}
              >
                <DeleteIcon />
              </IconButton>
            </div>
            <Accordion>
              <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                <Typography variant="caption">Advanced (override server preset)</Typography>
              </AccordionSummary>
              <AccordionDetails className={classes.details}>
                <Typography variant="caption">
                  Leave empty to use the server preset for this source type. Set a value only to
                  override it for this device.
                </Typography>
                <TextField
                  fullWidth={true}
                  label="Custom path"
                  placeholder="./uav_media/"
                  value={file['path'] ?? ''}
                  onChange={(e) => setFileField(index, 'path', e.target.value)}
                  helperText="Remote folder to read files from. Overrides the preset path."
                />
                <FormControl variant="outlined">
                  <InputLabel id={'download-type-label-' + index}>Download type</InputLabel>
                  <Select
                    labelId={'download-type-label-' + index}
                    label="Download type"
                    value={file['downloadType'] ?? ''}
                    onChange={(e) => setFileField(index, 'downloadType', e.target.value)}
                  >
                    <MenuItem value="">
                      <em>Use preset</em>
                    </MenuItem>
                    <MenuItem value="all">all</MenuItem>
                    <MenuItem value="lastFolder">lastFolder</MenuItem>
                    <MenuItem value="specific">specific</MenuItem>
                  </Select>
                </FormControl>
                <FormControl variant="outlined">
                  <InputLabel id={'delete-label-' + index}>Delete after download</InputLabel>
                  <Select
                    labelId={'delete-label-' + index}
                    label="Delete after download"
                    value={boolToSelect(file['delete'])}
                    onChange={(e) => setFileField(index, 'delete', selectToBool(e.target.value))}
                  >
                    <MenuItem value="">
                      <em>Use preset</em>
                    </MenuItem>
                    <MenuItem value="true">true</MenuItem>
                    <MenuItem value="false">false</MenuItem>
                  </Select>
                </FormControl>
                <FormControl variant="outlined">
                  <InputLabel id={'srv-download-label-' + index}>Service download</InputLabel>
                  <Select
                    labelId={'srv-download-label-' + index}
                    label="Service download"
                    value={boolToSelect(file['srvDownload'])}
                    onChange={(e) =>
                      setFileField(index, 'srvDownload', selectToBool(e.target.value))
                    }
                  >
                    <MenuItem value="">
                      <em>Use preset</em>
                    </MenuItem>
                    <MenuItem value="true">true</MenuItem>
                    <MenuItem value="false">false</MenuItem>
                  </Select>
                </FormControl>
              </AccordionDetails>
            </Accordion>
            <Divider />
          </Fragment>
        ))}

        <Button variant="contained" onClick={addFile}>
          Add files source
        </Button>
      </AccordionDetails>
    </Accordion>
  );
};

export default DeviceFilesEditor;
