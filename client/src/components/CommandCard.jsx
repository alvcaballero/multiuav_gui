import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';

import {
  Typography,
  Container,
  IconButton,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Button,
  Card,
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';

import makeStyles from '@mui/styles/makeStyles';
import { useNavigate } from 'react-router-dom';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import BaseCommandView from '../common/components/BaseCommandView';
import { useCatch } from '../reactHelper';

const useStyles = makeStyles((theme) => ({
  root: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '50%',
    top: '20%',
    transform: 'translateX(-50%)',
  },
  card: {
    pointerEvents: 'auto',
    width: theme.dimensions.popupMaxWidth,
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
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

const CommandCard = ({ id, onClose }) => {
  const classes = useStyles();

  const [item, setItem] = useState({});
  const [savedId, setSavedId] = useState(0);
  const limitCommands = 0;

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
      command = item;
    }

    command.deviceId = parseInt(id, 10);

    const response = await fetch('/api/commands/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(command),
    });

    if (response.ok) {
      console.log('ok');
    } else {
      throw Error(await response.text());
    }
  });

  const validate = () => savedId || (item && item.type);

  return (
    <div className={classes.root}>
      <Card elevation={3} className={classes.card}>
        <div className={classes.header}>
          <Typography variant="body2" color="textSecondary">
            Send Commands
          </Typography>
          <IconButton size="small" onClick={onClose} onTouchStart={onClose}>
            <CloseIcon fontSize="small" />
          </IconButton>
        </div>
        <Container maxWidth="xs" className={classes.container}>
          <Accordion defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">{'Command'}</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              {!limitCommands && !savedId && (
                <BaseCommandView deviceId={id} item={item} setItem={setItem} />
              )}
            </AccordionDetails>
          </Accordion>
          <div className={classes.buttons}>
            <Button type="button" color="primary" variant="outlined" onClick={onClose}>
              Cancel
            </Button>
            <Button
              type="button"
              color="primary"
              variant="contained"
              onClick={handleSend}
              disabled={!validate()}
            >
              Send
            </Button>
          </div>
        </Container>
      </Card>
    </div>
  );
};
export default CommandCard;
