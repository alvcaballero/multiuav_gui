import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Typography,
} from '@mui/material';

/** Confirmation for actions that can't be taken back (abort mission, land, kill motors). */
const ConfirmDialog = ({
  open,
  title,
  message,
  cancelLabel = 'Cancel',
  confirmLabel,
  onCancel,
  onConfirm,
}) => (
  <Dialog
    open={open}
    onClose={onCancel}
    maxWidth={false}
    slotProps={{ paper: { sx: { width: 440 } } }}
  >
    <DialogTitle>{title}</DialogTitle>
    <DialogContent>
      <Typography variant="body2">{message}</Typography>
    </DialogContent>
    <DialogActions>
      <Button variant="outlined" onClick={onCancel}>
        {cancelLabel}
      </Button>
      <Button variant="contained" color="error" onClick={onConfirm}>
        {confirmLabel}
      </Button>
    </DialogActions>
  </Dialog>
);

export default ConfirmDialog;
