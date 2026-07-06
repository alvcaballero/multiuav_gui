import React, { useState } from 'react';
import { useSelector } from 'react-redux';
import { useAsyncTask } from '../reactHelper';

import {
  Typography,
  Container,
  Paper,
  AppBar,
  Toolbar,
  IconButton,
  Table,
  TableContainer,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
  TextField,
  Chip,
  Box,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import EventBusyIcon from '@mui/icons-material/EventBusy';
import { useNavigate } from 'react-router-dom';
import { formatTime } from '../shared/formatter';

const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
  toolbarSpacer: {
    flexGrow: 1,
  },
  content: {
    overflow: 'auto',
    paddingTop: theme.spacing(2),
    paddingBottom: theme.spacing(2),
  },
  empty: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    gap: theme.spacing(1),
    padding: theme.spacing(6),
    color: theme.palette.text.secondary,
  },
}));

const today = () => new Date().toISOString().slice(0, 10);

const EventsPage = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();

  const [date, setDate] = useState(() => today());
  const [items, setItems] = useState(null);
  const devices = useSelector((state) => state.devices.items);

  useAsyncTask(async () => {
    setItems(null);
    const params = new URLSearchParams({ from: `${date}T00:00:00`, to: `${date}T23:59:59` });
    const response = await fetch(`/api/events?${params}`);
    if (response.ok) {
      setItems(await response.json());
    } else {
      throw Error(await response.text());
    }
  }, [date]);

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton color="inherit" edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6">Events</Typography>
          <div className={classes.toolbarSpacer} />
          <TextField
            type="date"
            size="small"
            variant="outlined"
            value={date}
            onChange={(e) => setDate(e.target.value)}
            slotProps={{ htmlInput: { max: today() } }}
          />
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container maxWidth="sm">
          <Paper>
            <TableContainer>
              <Table size="small">
                <TableHead>
                  <TableRow>
                    <TableCell>UAV</TableCell>
                    <TableCell>Type</TableCell>
                    <TableCell>Time</TableCell>
                    <TableCell>Message</TableCell>
                  </TableRow>
                </TableHead>
                <TableBody>
                  {items &&
                    items.map((event) => (
                      <TableRow key={event.id} hover>
                        <TableCell>
                          {event.deviceId
                            ? (devices[event.deviceId]?.name ?? event.deviceId)
                            : 'GCS'}
                        </TableCell>
                        <TableCell>
                          <Chip label={event.type} size="small" variant="outlined" />
                        </TableCell>
                        <TableCell>{formatTime(event.eventTime, 'minutes')}</TableCell>
                        <TableCell>{event.attributes?.message}</TableCell>
                      </TableRow>
                    ))}
                </TableBody>
              </Table>
            </TableContainer>
            {items && items.length === 0 && (
              <Box className={classes.empty}>
                <EventBusyIcon fontSize="large" />
                <Typography variant="body2">No events for this date</Typography>
              </Box>
            )}
          </Paper>
        </Container>
      </div>
    </div>
  );
};

export default EventsPage;
