import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';

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
import { prefixString } from '../common/stringUtils';
import PositionValue from '../components/PositionValue';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import BaseCommandView from '../common/components/BaseCommandView';
import { useCatch } from '../reactHelper';
import SquareMove from './SquareMove';

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

const DevicePage = () => {
  const classes = useStyles();
  const navigate = useNavigate();

  const { id } = useParams();

  const [item, setItem] = useState();
  const [itemc, setItemc] = useState({});
  const deviceposition = useSelector((state) => state.data.positions);
  const [savedId, setSavedId] = useState(0);
  const limitCommands = 0;
  useEffect(() => {
    if (id) {
      setItem(deviceposition[id]);
    }
  }, [id, deviceposition]);

  const deviceName = useSelector((state) => {
    if (item) {
      const device = state.devices.items[item.deviceId];
      if (device) {
        return device.name;
      }
    }
    return null;
  });

  const handleSend = useCatch(async () => {
    let command;
    if (savedId) {
      const response = await fetch(`/api/commands/${savedId}`);
      if (response.ok) {
        command = await response.json();
      } else {
        throw Error(await response.text());
      }
    } else {
      command = itemc;
    }

    command.deviceId = parseInt(id, 10);

    const response = await fetch('/api/commands/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(command),
    });

    if (response.ok) {
      navigate(-1);
    } else {
      throw Error(await response.text());
    }
  });

  const validate = () => savedId || (itemc && itemc.type);

  return (
    <div className={classes.root}>
      <AppBar position='sticky' color='inherit'>
        <Toolbar>
          <IconButton color='inherit' edge='start' sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant='h6'>{deviceName}</Typography>
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container maxWidth='sm'>
          <Paper>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>stateName</TableCell>
                  <TableCell>sharedName</TableCell>
                  <TableCell>stateValue</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {item &&
                  Object.getOwnPropertyNames(item)
                    .filter((it) => it !== 'attributes')
                    .map((property) => (
                      <TableRow key={property}>
                        <TableCell>{property}</TableCell>
                        <TableCell>
                          <strong>{prefixString('position', property)}</strong>
                        </TableCell>
                        <TableCell>
                          <PositionValue position={item} property={property} />
                        </TableCell>
                      </TableRow>
                    ))}
                {item &&
                  Object.getOwnPropertyNames(item.attributes).map((attribute) => (
                    <TableRow key={attribute}>
                      <TableCell>{attribute}</TableCell>
                      <TableCell>
                        <strong>
                          {prefixString('position', attribute) || prefixString('device', attribute)}
                        </strong>
                      </TableCell>
                      <TableCell>
                        <PositionValue position={item} attribute={attribute} />
                      </TableCell>
                    </TableRow>
                  ))}
              </TableBody>
            </Table>
          </Paper>
        </Container>
      </div>
      <div>
        {item && item.attributes && (
          <div>
            <SquareMove front_view={true} data={item.attributes.obstacle_info}></SquareMove>
            <div></div>
            <SquareMove front_view={false} data={item.attributes.obstacle_info}></SquareMove>
          </div>
        )}
      </div>
      <div>
        <Container maxWidth='xs' className={classes.container}>
          <Accordion defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant='subtitle1'>{'Command'}</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              {!limitCommands && !savedId && (
                <BaseCommandView deviceId={id} item={itemc} setItem={setItemc} />
              )}
            </AccordionDetails>
          </Accordion>
          <div className={classes.buttons}>
            <Button type='button' color='primary' variant='outlined' onClick={() => navigate(-1)}>
              Cancel
            </Button>
            <Button
              type='button'
              color='primary'
              variant='contained'
              onClick={handleSend}
              disabled={!validate()}
            >
              Send
            </Button>
          </div>
        </Container>
      </div>
    </div>
  );
};

export default DevicePage;
