import { useEffect, useState } from 'react';
import { useSelector } from 'react-redux';

import {
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  IconButton,
  List,
  ListItemButton,
  ListItemText,
  Typography,
  Button,
  Box,
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';

import { makeStyles } from 'tss-react/mui';

import BaseCommandView from '../../common/components/BaseCommandView';
import { useCatch } from '../../reactHelper';

const useStyles = makeStyles()((theme) => ({
  title: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    borderBottom: `1px solid ${theme.palette.divider}`,
  },
  body: {
    display: 'flex',
    padding: 0,
    minHeight: 360,
  },
  list: {
    width: 250,
    flex: 'none',
    overflowY: 'auto',
    maxHeight: 440,
    borderRight: `1px solid ${theme.palette.divider}`,
    padding: 0,
  },
  item: {
    borderLeft: '2px solid transparent',
    '&.Mui-selected': {
      borderLeftColor: theme.palette.primary.main,
    },
  },
  code: {
    fontFamily: 'ui-monospace, Menlo, monospace',
    fontSize: 11,
  },
  detail: {
    flex: 1,
    minWidth: 0,
    padding: theme.spacing(2),
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1.5),
  },
  params: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr',
    gap: theme.spacing(1.5),
  },
  payload: {
    margin: 0,
    padding: theme.spacing(1),
    background: theme.palette.action.hover,
    fontFamily: 'ui-monospace, Menlo, monospace',
    fontSize: 12,
    overflow: 'auto',
  },
  footer: {
    borderTop: `1px solid ${theme.palette.divider}`,
    padding: theme.spacing(1, 2),
  },
}));

const defaultAttributes = (params = []) =>
  Object.fromEntries(params.filter((p) => p.default != null).map((p) => [p.id, p.default]));

/**
 * Send-command dialog. Lists the commands this device supports, as GET /api/commands/types
 * resolved them (label, description, params with units/ranges/defaults). `hidden` ones are
 * left out: their payload is built by another flow (e.g. a mission).
 * By default it POSTs the command itself; pass `onSend(type, attributes)` to send it through the
 * caller instead (DevicePage does, to log it in its stream and show its toast).
 */
const CommandCard = ({ id, onClose, onSend }) => {
  const { classes } = useStyles();
  const deviceName = useSelector((state) => state.devices.items[id]?.name);

  const [commands, setCommands] = useState([]);
  const [item, setItem] = useState({ attributes: {} });

  useEffect(() => {
    const controller = new AbortController();
    fetch(`/api/commands/types?${new URLSearchParams({ deviceId: id })}`, {
      signal: controller.signal,
    })
      .then((response) => (response.ok ? response.json() : []))
      .then((all) => setCommands(all.filter((c) => !c.hidden)))
      .catch(() => {});
    return () => controller.abort();
  }, [id]);

  const command = commands.find((c) => c.type === item.type);

  const select = (selected) =>
    setItem({ type: selected.type, attributes: defaultAttributes(selected.params) });

  const payload = { deviceId: parseInt(id, 10), type: item.type, attributes: item.attributes };

  const handleSend = useCatch(async () => {
    if (onSend) {
      await onSend(item.type, item.attributes);
    } else {
      const response = await fetch('/api/commands/send', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ...payload, textChannel: item.textChannel }),
      });
      if (!response.ok) {
        throw Error(await response.text());
      }
      // A rejected command (e.g. a param out of range) comes back as 200 { state: 'error', msg }.
      const result = await response.json();
      if (result?.state === 'error') throw Error(result.msg);
    }
    onClose();
  });

  const hasParams = Boolean(command?.params?.length);

  return (
    <Dialog open onClose={onClose} maxWidth={false} slotProps={{ paper: { sx: { width: 760 } } }}>
      <DialogTitle className={classes.title}>
        Send command{deviceName ? ` · ${deviceName}` : ''}
        <IconButton size="small" onClick={onClose} aria-label="Close">
          <CloseIcon fontSize="small" />
        </IconButton>
      </DialogTitle>
      <DialogContent className={classes.body}>
        <List className={classes.list} dense>
          {commands.map((c) => (
            <ListItemButton
              key={c.type}
              selected={c.type === item.type}
              onClick={() => select(c)}
              className={classes.item}
            >
              <ListItemText
                primary={c.label ?? c.type}
                secondary={`${c.type} · ${c.requires ? `requires ${c.requires}` : 'always'}`}
                slotProps={{ secondary: { className: classes.code } }}
              />
            </ListItemButton>
          ))}
        </List>
        <Box className={classes.detail}>
          {command ? (
            <>
              <Typography variant="h6">{command.label ?? command.type}</Typography>
              {command.description && (
                <Typography variant="body2" color="textSecondary">
                  {command.description}
                </Typography>
              )}
              {hasParams && (
                <div className={classes.params}>
                  <BaseCommandView item={item} setItem={setItem} params={command.params} />
                </div>
              )}
              <Typography variant="overline" color="textSecondary">
                POST /api/commands/send
              </Typography>
              <pre className={classes.payload}>{JSON.stringify(payload, null, 2)}</pre>
            </>
          ) : (
            <Typography variant="body2" color="textSecondary">
              Pick a command from the list.
            </Typography>
          )}
        </Box>
      </DialogContent>
      <DialogActions className={classes.footer}>
        <Typography variant="body2" color="textSecondary" sx={{ mr: 'auto' }}>
          Showing {commands.length} commands this UAV supports.
        </Typography>
        <Button variant="outlined" onClick={onClose}>
          Cancel
        </Button>
        <Button variant="contained" onClick={handleSend} disabled={!item.type}>
          Send
        </Button>
      </DialogActions>
    </Dialog>
  );
};

export default CommandCard;
