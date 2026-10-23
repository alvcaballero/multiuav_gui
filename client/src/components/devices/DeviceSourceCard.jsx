import { makeStyles } from 'tss-react/mui';
import { IconButton, Paper, Tooltip, Typography } from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';

const useStyles = makeStyles()((theme) => ({
  card: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1.5),
    padding: theme.spacing(1, 1.5, 1.5),
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
}));

// One entry of a device's source list (a camera, a file source): a titled card with its delete
// button. Shared by DeviceCameraEditor and DeviceFilesEditor; the fields are the children.
const DeviceSourceCard = ({ title, onRemove, children }) => {
  const { classes } = useStyles();
  return (
    <Paper variant="outlined" className={classes.card}>
      <div className={classes.header}>
        <Typography variant="subtitle2">{title}</Typography>
        <Tooltip title="Remove">
          <IconButton size="small" onClick={onRemove} aria-label={`Remove ${title}`}>
            <DeleteIcon fontSize="small" />
          </IconButton>
        </Tooltip>
      </div>
      {children}
    </Paper>
  );
};

export default DeviceSourceCard;
