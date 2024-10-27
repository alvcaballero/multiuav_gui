import React, { useState } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import SelectField from '../common/components/SelectField';
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
  FormGroup,
  FormControlLabel,
  Checkbox,
} from '@mui/material';
import { IconButton, Menu, MenuItem, useMediaQuery, useTheme } from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import MoreVertIcon from '@mui/icons-material/MoreVert';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import LinkIcon from '@mui/icons-material/Link';
import { useEffectAsync } from '../reactHelper';
import PageLayout from '../common/components/PageLayout';
import SettingsMenu from './components/SettingsMenu';
import TableShimmer from '../common/components/TableShimmer';
import SearchHeader, { filterByKeyword } from './components/SearchHeader';
import { formatTime } from '../common/formatter';
import useSettingsStyles from './common/useSettingsStyles';
import RemoveDialog from '../components/RemoveDialog';
import useQuery from './common/useQuery';

const SettingsCategoryPageEdit = () => {
  const classes = useSettingsStyles();
  const navigate = useNavigate();

  const query = useQuery();
  const uniqueId = query.get('uniqueId');

  const [item, setItem] = useState(uniqueId ? { uniqueId } : null);

  const [timestamp, setTimestamp] = useState(Date.now());
  const [items, setItems] = useState([]);
  const [searchKeyword, setSearchKeyword] = useState('');
  const [showAll, setShowAll] = useState(false);
  const [loading, setLoading] = useState(false);
  const [removing, setRemoving] = useState(false);
  const [myCategory, setMyCategory] = useState(null);

  useEffectAsync(async () => {
    setLoading(true);
    try {
      const query = new URLSearchParams({ all: showAll });
      const response = await fetch(`/api/category`);
      if (response.ok) {
        setItems(await response.json());
      } else {
        throw Error(await response.text());
      }
    } finally {
      setLoading(false);
    }
  }, [timestamp, showAll]);

  const handleEdit = (item) => {
    navigate(`/settings/category/${item}`);
  };

  const handleRemove = (item) => {
    setMyCategory(item);
    setRemoving(true);
  };

  const hamdleRemoveResult = (result) => {
    setMyCategory(null);
    setRemoving(false);
  };

  return (
    <EditItemView
      endpoint="devices"
      item={item}
      setItem={setItem}
      validate={validate}
      menu={<SettingsMenu />}
      breadcrumbs={['settingsTitle', 'sharedDevice']}
    >
      {item && (
        <>
          <Accordion defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">telemetry - topics</Typography>
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

          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Camera Stream</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <Typography variant="caption">Source example:main</Typography>
              {item.camera &&
                item.camera.map((action_key, index_ac, list_ac) => (
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
                                let mycam = JSON.parse(JSON.stringify(cam));
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
                                let mycam = JSON.parse(JSON.stringify(cam));
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
                        onClick={() => Remove_camera(index_ac)}
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
                The url format sftp://user:password@Ip:port It can be compatible for sftp or ftp protovol
              </Typography>
              {item.files &&
                item.files.map((action_key, index_ac, list_ac) => (
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
                                let mycam = JSON.parse(JSON.stringify(cam));
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
                          onChange={(e) =>
                            setItem({
                              ...item,
                              files: item.files.map((cam, cam_ind) => {
                                let mycam = JSON.parse(JSON.stringify(cam));
                                index_ac == cam_ind ? (mycam['url'] = e.target.value) : null;
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
                        onClick={() => Remove_file(index_ac)}
                        className={classes.negative}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </div>
                    <Divider></Divider>
                  </Fragment>
                ))}

              <Button variant="contained" onClick={addNewFile}>
                Add files source
              </Button>
            </AccordionDetails>
          </Accordion>
        </>
      )}
    </EditItemView>
  );
};

export default SettingsCategoryPageEdit;
