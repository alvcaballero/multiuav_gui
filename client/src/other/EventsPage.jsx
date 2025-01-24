import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { useEffectAsync } from '../reactHelper';

import {
  Typography,
  Container,
  Paper,
  AppBar,
  Toolbar,
  IconButton,
  Table,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Button,
} from '@mui/material';
import makeStyles from '@mui/styles/makeStyles';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { useNavigate, useParams } from 'react-router-dom';

const useStyles = makeStyles((theme) => ({
  root: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
  content: {
    overflow: 'auto',
    paddingTop: theme.spacing(2),
    paddingBottom: theme.spacing(2),
  },
  buttons: {
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(2),
    display: 'flex',
    justifyContent: 'space-evenly',
    '& > *': {
      flexBasis: '33%',
    },
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
}));

const EventsPage = () => {
  const classes = useStyles();
  const navigate = useNavigate();

  const [item, setItem] = useState([]);
  const [itemdevice, setItemdevice] = useState();
  const events = useSelector((state) => state.events.items);
  const devices = useSelector((state) => state.devices.items);
  // useEffect(() => {
  //   setItem(events);
  // }, [events]);
  useEffect(() => {
    console.log(item);
  }, [item]);
  useEffect(() => {
    setItemdevice({ ...devices, '-1': { name: 'GCS' } });
    //console.log('my devices');
    //console.log({ ...devices, '-1': { name: 'GCS' } });
  }, [devices]);

  useEffectAsync(async () => {
    const response = await fetch('/api/events');
    if (response.ok) {
      setItem(await response.json());
    } else {
      throw Error(await response.text());
    }
  }, []);


  const deviceName = useSelector((state) => {
    if (item) {
      const device = state.devices.items[item.deviceId];
      if (device) {
        return device.name;
      }
    }
    return null;
  });

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton color="inherit" edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6">Events</Typography>
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container maxWidth="sm">
          <Paper>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>UAV</TableCell>
                  <TableCell>Type</TableCell>
                  <TableCell>Fecha</TableCell>
                  <TableCell>Valor</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {item &&
                  item.map((event) => (
                    <TableRow key={event.id}>
                      <TableCell>{itemdevice[event.deviceId].name}</TableCell>
                      <TableCell>{event.type}</TableCell>
                      <TableCell>{event.eventTime}</TableCell>
                      <TableCell>{event.attributes.message}</TableCell>
                    </TableRow>
                  ))}
              </TableBody>
            </Table>
          </Paper>
        </Container>
      </div>
    </div>
  );
};

export default EventsPage;
