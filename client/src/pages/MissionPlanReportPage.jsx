import React, { useState } from 'react';
import { useDispatch } from 'react-redux';
import { useAsyncTask } from '../reactHelper';

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
  Chip,
  Tooltip,
} from '@mui/material';

import { makeStyles } from 'tss-react/mui';

import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import MapIcon from '@mui/icons-material/Map';
import { useNavigate } from 'react-router-dom';
import { formatTime } from '../shared/formatter';
import { loadPlanToEditor } from '../services/missionPlanLoader';

const useStyles = makeStyles()((theme) => ({
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
}));

const COLUMNS_ARRAY = ['id', 'name', 'source', 'createdAt', 'tasks'];

const MissionPlanReportPage = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();

  const [plans, setPlans] = useState(null);

  const formatValue = (item, key) => {
    switch (key) {
      case 'name':
        return item.name ?? '-';
      case 'source':
        return (
          <Chip
            label={item.source.toUpperCase()}
            color={item.source === 'automatic' ? 'info' : 'default'}
            size="small"
          />
        );
      case 'createdAt':
        return formatTime(item.createdAt, 'minutes');
      case 'tasks': {
        // Plans store either a task graph (tasks[]) or legacy routes (route[]).
        const { tasks, route } = item.missionData ?? {};
        return (tasks ?? route ?? []).length;
      }
      default:
        return item[key];
    }
  };

  const handleLoad = async (planId) => {
    if (await loadPlanToEditor(planId, dispatch)) navigate('/');
  };

  useAsyncTask(async () => {
    const response = await fetch('/api/missions/plans');
    if (response.ok) {
      const myPlans = await response.json();
      myPlans.sort((a, b) => new Date(b.createdAt) - new Date(a.createdAt));
      setPlans(myPlans);
    } else {
      throw Error(await response.text());
    }
  }, []);

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton color="inherit" edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6">Mission Plans</Typography>
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container fixed>
          <Paper>
            <Table>
              <TableHead>
                <TableRow>
                  {COLUMNS_ARRAY.map((key) => (
                    <TableCell key={`${key}x`}>{key.toUpperCase()}</TableCell>
                  ))}
                  <TableCell key="loadx">Load</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {plans &&
                  plans.map((item) => (
                    <TableRow key={`${item.id}_`}>
                      {COLUMNS_ARRAY.map((key) => (
                        <TableCell key={key}>{formatValue(item, key)}</TableCell>
                      ))}
                      <TableCell>
                        <Tooltip title="Load plan in map">
                          <IconButton size="small" onClick={() => handleLoad(item.id)}>
                            <MapIcon fontSize="small" />
                          </IconButton>
                        </Tooltip>
                      </TableCell>
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

export default MissionPlanReportPage;
