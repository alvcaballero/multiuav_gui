import React, { useState } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import SelectField from '../common/components/SelectField';
import {
  IconButton,
  Button,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  Table,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
  Box,
} from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import DeleteIcon from '@mui/icons-material/Delete';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import { useEffectAsync } from '../reactHelper';
import SettingsMenu from './components/SettingsMenu';
import useSettingsStyles from './common/useSettingsStyles';
import useQuery from '../common/useQuery';
import EditItemView from './components/EditItemView';

const SettingsCategoryPageEdit = () => {
  const { classes } = useSettingsStyles();

  const query = useQuery();
  const uniqueId = query.get('uniqueId');

  const [item, setItem] = useState(uniqueId ? { uniqueId } : null);
  const [itemMsg, setItemMsg] = useState(uniqueId ? { uniqueId } : null);
  const [loading, setLoading] = useState(false);
  const [showAll, setShowAll] = useState(false);
  const [typeMsgMenu, setTypeMsgMenu] = useState(true);
  const [selectTypeMsgMenu, setSelectTypeMsgMenu] = useState(true);

  useEffectAsync(async () => {
    setLoading(true);
    try {
      const response2 = await fetch('/api/category/messages');
      if (response2.ok) {
        const data1 = await response2.json();
        console.log(data1);
        setItemMsg(data1);
      } else {
        throw Error(await response2.text());
      }
    } finally {
      setLoading(false);
    }
  }, [showAll]);

  const addnewTopic = () => {
    const newItem = JSON.parse(JSON.stringify(item));
    if (!newItem.topics) {
      newItem.topics = {};
    }
    if (!newItem.topics[selectTypeMsgMenu]) {
      newItem.topics[selectTypeMsgMenu] = { name: '', messageType: itemMsg.topics[selectTypeMsgMenu][0] };
    }
    setItem(newItem);
    setSelectTypeMsgMenu(null);
    setTypeMsgMenu(!typeMsgMenu);
  };

  const addnewService = () => {
    const newItem = JSON.parse(JSON.stringify(item));
    if (!newItem.services) {
      newItem.services = {};
    }
    if (!newItem.services[selectTypeMsgMenu]) {
      newItem.services[selectTypeMsgMenu] = { name: '', messageType: itemMsg.services[selectTypeMsgMenu][0] };
    }
    setItem(newItem);
    setSelectTypeMsgMenu(null);
    setTypeMsgMenu(!typeMsgMenu);
  };

  const changeMsgType = (value, key, type) => {
    const newItem = JSON.parse(JSON.stringify(item));
    newItem[type][key].messageType = value;
    setItem(newItem);
  };

  const removeElement = (key, type) => {
    const newItem = JSON.parse(JSON.stringify(item));
    delete newItem[type][key];
    setItem(newItem);
  };

  const validate = () => item && item.topics && item.services;

  return (
    <EditItemView
      endpoint="category"
      item={item}
      setItem={setItem}
      validate={validate}
      menu={<SettingsMenu />}
      breadcrumbs={['Category', 'M300']}
    >
      {!loading && item && (
        <>
          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Telemetry - topics</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <Table className={classes.table}>
                <TableHead>
                  <TableRow>
                    <TableCell>{'Topic'}</TableCell>
                    <TableCell>{'Name'}</TableCell>
                    <TableCell>{'Message'}</TableCell>
                    <TableCell className={classes.columnAction} />
                  </TableRow>
                </TableHead>
                <TableBody>
                  {Object.keys(item.topics).map((key) => (
                    <TableRow key={key}>
                      <TableCell>{key}</TableCell>
                      <TableCell>
                        <TextField value={item.topics[key].name} />
                      </TableCell>
                      <TableCell>
                        <SelectField
                          data={itemMsg.topics[key]}
                          value={item.topics[key].messageType}
                          keyGetter={(e) => e}
                          titleGetter={(e) => e}
                          onChange={(e) => changeMsgType(e.target.value, key, 'topics')}
                          emptyValue={null}
                        />
                      </TableCell>
                      <TableCell className={classes.columnAction} padding="none">
                        <Tooltip title="Remove">
                          <IconButton size="small" onClick={() => removeElement(key, 'topics')}>
                            <DeleteIcon fontSize="small" />
                          </IconButton>
                        </Tooltip>
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>

              <Box textAlign="center">
                {typeMsgMenu ? (
                  <Button variant="contained" onClick={() => setTypeMsgMenu(!typeMsgMenu)}>
                    add a telemetry
                  </Button>
                ) : (
                  <div>
                    <Typography variant="subtitle1">Add Telemetry</Typography>
                    <SelectField
                      fullWidth={true}
                      data={Object.keys(itemMsg.topics)}
                      emptyValue={null}
                      value={selectTypeMsgMenu}
                      onChange={(e) => setSelectTypeMsgMenu(e.target.value)}
                      keyGetter={(e) => e}
                      titleGetter={(e) => e}
                    />
                    <div>
                      <Button onClick={() => setTypeMsgMenu(!typeMsgMenu)}>Cancel</Button>
                      <Button onClick={() => addnewTopic()}>Add</Button>
                    </div>
                  </div>
                )}
              </Box>
            </AccordionDetails>
          </Accordion>
          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Commands - Services</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <Table className={classes.table}>
                <TableHead>
                  <TableRow>
                    <TableCell>{'Services'}</TableCell>
                    <TableCell>{'Name'}</TableCell>
                    <TableCell>{'Message'}</TableCell>
                    <TableCell className={classes.columnAction} />
                  </TableRow>
                </TableHead>
                <TableBody>
                  {Object.keys(item.services).map((key) => (
                    <TableRow key={key}>
                      <TableCell>{key}</TableCell>
                      <TableCell>
                        <TextField value={item.services[key].name} />
                      </TableCell>
                      <TableCell>
                        <SelectField
                          data={itemMsg.services[key]}
                          value={item.services[key].serviceType}
                          keyGetter={(e) => e}
                          titleGetter={(e) => e}
                          onChange={(e) => changeMsgType(e.target.value, key, 'services')}
                          emptyValue={null}
                        />
                      </TableCell>
                      <TableCell className={classes.columnAction} padding="none">
                        <Tooltip title="Remove">
                          <IconButton size="small" onClick={() => removeElement(key, 'services')}>
                            <DeleteIcon fontSize="small" />
                          </IconButton>
                        </Tooltip>
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>

              <Box textAlign="center">
                {typeMsgMenu ? (
                  <Button variant="contained" onClick={() => setTypeMsgMenu(!typeMsgMenu)}>
                    add a telemetry
                  </Button>
                ) : (
                  <div>
                    <Typography variant="subtitle1">Add command</Typography>
                    <SelectField
                      fullWidth={true}
                      data={Object.keys(itemMsg.services)}
                      emptyValue={null}
                      value={selectTypeMsgMenu}
                      onChange={(e) => setSelectTypeMsgMenu(e.target.value)}
                      keyGetter={(e) => e}
                      titleGetter={(e) => e}
                    />
                    <div>
                      <Button onClick={() => setTypeMsgMenu(!typeMsgMenu)}>Cancel</Button>
                      <Button onClick={() => addnewService()}>Add</Button>
                    </div>
                  </div>
                )}
              </Box>
            </AccordionDetails>
          </Accordion>
        </>
      )}
    </EditItemView>
  );
};

export default SettingsCategoryPageEdit;
