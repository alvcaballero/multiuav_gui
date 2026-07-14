import React, { useState, Fragment } from 'react';
import { makeStyles } from 'tss-react/mui';

import CloseIcon from '@mui/icons-material/Close';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Divider,
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
} from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';
import SelectField from '../../shared/components/SelectField';
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
  const removeCamera = (index) => {
    let auxcamera = structuredClone(item.camera);
    auxcamera.splice(index, 1);
    setItem({ ...item, camera: auxcamera });
  };
  function addNewcamera() {
    let auxcamera = structuredClone(item.camera);
    auxcamera.push({ type: 'WebRTC', source: '' });
    setItem({ ...item, camera: auxcamera });
  }
  const removeFile = (index) => {
    let auxcamera = structuredClone(item.files);
    auxcamera.splice(index, 1);
    setItem({ ...item, files: auxcamera });
  };
  const addNewFile = () => {
    let auxfile = structuredClone(item.files);
    auxfile.push({ type: 'onboard_computer', url: '' });
    setItem({ ...item, files: auxfile });
  };
  // Set/clear a single field on the file entry at `index`. An empty value removes
  // the override so the server preset (devices.yaml) keeps applying for that field.
  const setFileField = (index, field, value) => {
    setItem({
      ...item,
      files: item.files.map((file, fileIndex) => {
        if (fileIndex !== index) return file;
        let myfile = structuredClone(file);
        if (value === '' || value === undefined) delete myfile[field];
        else myfile[field] = value;
        return myfile;
      }),
    });
  };
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
            <Accordion>
              <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                <Typography variant="subtitle1">Camera Stream</Typography>
              </AccordionSummary>
              <AccordionDetails className={classes.details}>
                <Typography variant="caption">Source example:main</Typography>
                {item.camera &&
                  item.camera.map((action_key, index_ac) => (
                    <Fragment key={'fragment-action-' + index_ac}>
                      <Typography variant="subtitle1" className={classes.attributeName}>
                        {'Camera ' + index_ac}
                      </Typography>
                      <div>
                        <FormControl variant="outlined">
                          <InputLabel id="demo-simple-select-outlined-label">CameraType</InputLabel>
                          <Select
                            labelId="demo-simple-select-outlined-label"
                            id="demo-simple-select-outlined"
                            value={action_key['type']}
                            label="type"
                            onChange={(e) =>
                              setItem({
                                ...item,
                                camera: item.camera.map((cam, cam_ind) => {
                                  let mycam = structuredClone(cam);
                                  index_ac == cam_ind ? (mycam['type'] = e.target.value) : null;
                                  return mycam;
                                }),
                              })
                            }
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
                            value={action_key['source']}
                            onChange={(e) =>
                              setItem({
                                ...item,
                                camera: item.camera.map((cam, cam_ind) => {
                                  let mycam = structuredClone(cam);
                                  index_ac == cam_ind ? (mycam['source'] = e.target.value) : null;
                                  return mycam;
                                }),
                              })
                            }
                          />
                        </div>
                        <IconButton
                          sx={{
                            py: 0,
                            pr: 2,
                            marginLeft: 'auto',
                          }}
                          onClick={() => removeCamera(index_ac)}
                          className={classes.negative}
                        >
                          <DeleteIcon />
                        </IconButton>
                      </div>
                      <Divider></Divider>
                    </Fragment>
                  ))}

                <Button variant="contained" onClick={addNewcamera}>
                  Add camera source
                </Button>
              </AccordionDetails>
            </Accordion>
            <Accordion>
              <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                <Typography variant="subtitle1">Device Files Resurces</Typography>
              </AccordionSummary>
              <AccordionDetails className={classes.details}>
                <Typography variant="caption">
                  The url format sftp://user:password@Ip:port It can be compatible for sftp or ftp
                  protovol
                </Typography>
                {item.files &&
                  item.files.map((action_key, index_ac) => (
                    <Fragment key={'fragment-action-file' + index_ac}>
                      <Typography variant="subtitle1" className={classes.attributeName}>
                        {'File ' + index_ac}
                      </Typography>
                      <div>
                        <FormControl variant="outlined">
                          <InputLabel id="demo-simple-select-outlined-label">CameraType</InputLabel>
                          <Select
                            labelId="demo-simple-select-outlined-label"
                            id="demo-simple-select-outlined"
                            value={action_key['type']}
                            label="type"
                            onChange={(e) =>
                              setItem({
                                ...item,
                                files: item.files.map((cam, cam_ind) => {
                                  let mycam = structuredClone(cam);
                                  index_ac == cam_ind ? (mycam['type'] = e.target.value) : null;
                                  return mycam;
                                }),
                              })
                            }
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
                            value={action_key['url']}
                            onChange={(e) => setFileField(index_ac, 'url', e.target.value)}
                          />
                        </div>
                        <IconButton
                          sx={{
                            py: 0,
                            pr: 2,
                            marginLeft: 'auto',
                          }}
                          onClick={() => removeFile(index_ac)}
                          className={classes.negative}
                        >
                          <DeleteIcon />
                        </IconButton>
                      </div>
                      <Accordion>
                        <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                          <Typography variant="caption">
                            Advanced (override server preset)
                          </Typography>
                        </AccordionSummary>
                        <AccordionDetails className={classes.details}>
                          <Typography variant="caption">
                            Leave empty to use the server preset for this source type. Set a value
                            only to override it for this device.
                          </Typography>
                          <TextField
                            fullWidth={true}
                            label="Custom path"
                            placeholder="./uav_media/"
                            value={action_key['path'] ?? ''}
                            onChange={(e) => setFileField(index_ac, 'path', e.target.value)}
                            helperText="Remote folder to read files from. Overrides the preset path."
                          />
                          <FormControl variant="outlined">
                            <InputLabel id={'download-type-label-' + index_ac}>
                              Download type
                            </InputLabel>
                            <Select
                              labelId={'download-type-label-' + index_ac}
                              label="Download type"
                              value={action_key['downloadType'] ?? ''}
                              onChange={(e) =>
                                setFileField(index_ac, 'downloadType', e.target.value)
                              }
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
                            <InputLabel id={'delete-label-' + index_ac}>
                              Delete after download
                            </InputLabel>
                            <Select
                              labelId={'delete-label-' + index_ac}
                              label="Delete after download"
                              value={
                                action_key['delete'] === undefined
                                  ? ''
                                  : String(action_key['delete'])
                              }
                              onChange={(e) =>
                                setFileField(
                                  index_ac,
                                  'delete',
                                  e.target.value === '' ? '' : e.target.value === 'true',
                                )
                              }
                            >
                              <MenuItem value="">
                                <em>Use preset</em>
                              </MenuItem>
                              <MenuItem value="true">true</MenuItem>
                              <MenuItem value="false">false</MenuItem>
                            </Select>
                          </FormControl>
                          <FormControl variant="outlined">
                            <InputLabel id={'srv-download-label-' + index_ac}>
                              Service download
                            </InputLabel>
                            <Select
                              labelId={'srv-download-label-' + index_ac}
                              label="Service download"
                              value={
                                action_key['srvDownload'] === undefined
                                  ? ''
                                  : String(action_key['srvDownload'])
                              }
                              onChange={(e) =>
                                setFileField(
                                  index_ac,
                                  'srvDownload',
                                  e.target.value === '' ? '' : e.target.value === 'true',
                                )
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

                <Button variant="contained" onClick={addNewFile}>
                  Add files source
                </Button>
              </AccordionDetails>
            </Accordion>
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
