import { useState, useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { Box, Typography, Button, Stack, Chip, TextField, Collapse, Alert } from '@mui/material';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import { chatActions } from '../../store/chat';
import { sendToolApprovalResponse } from '../../services/toolApproval';

const secondsLeft = (expiresAt) =>
  Math.max(0, Math.floor((new Date(expiresAt).getTime() - Date.now()) / 1000));

const formatCountdown = (seconds) =>
  `${Math.floor(seconds / 60)}:${String(seconds % 60).padStart(2, '0')}`;

/**
 * One parked tool call awaiting a decision.
 *
 * Renders the input EXACTLY as it will execute — the operator is approving this
 * value, not the intent behind it, so nothing here is summarised or reformatted.
 */
const ApprovalRequest = ({ request, chatId, onResponded }) => {
  const [remaining, setRemaining] = useState(() => secondsLeft(request.expiresAt));
  const [denying, setDenying] = useState(false);
  const [reason, setReason] = useState('');
  const [sending, setSending] = useState(false);
  const [error, setError] = useState(null);

  useEffect(() => {
    const tick = setInterval(() => setRemaining(secondsLeft(request.expiresAt)), 1000);
    return () => clearInterval(tick);
  }, [request.expiresAt]);

  const expired = remaining === 0;

  const respond = (optionId) => {
    setSending(true);
    setError(null);
    try {
      sendToolApprovalResponse(chatId, [
        { requestId: request.requestId, optionId, text: reason || undefined },
      ]);
      onResponded?.(request.requestId);
    } catch (err) {
      setError(err.message);
      setSending(false);
    }
  };

  return (
    <Box
      sx={{
        border: '2px solid',
        borderColor: expired ? 'grey.400' : 'error.main',
        borderRadius: '8px',
        bgcolor: expired ? 'grey.50' : '#fff8f6',
        p: 1.5,
        mb: 1,
      }}
    >
      <Stack direction="row" alignItems="center" spacing={1} sx={{ mb: 1 }}>
        <WarningAmberIcon fontSize="small" color={expired ? 'disabled' : 'error'} />
        <Typography variant="subtitle2" sx={{ fontWeight: 700, flexGrow: 1 }}>
          {request.toolName}
        </Typography>
        <Chip
          size="small"
          label={expired ? 'Expired' : formatCountdown(remaining)}
          color={expired ? 'default' : 'error'}
          variant="outlined"
        />
      </Stack>

      <Typography variant="caption" color="text.secondary">
        This command will be executed exactly as shown:
      </Typography>
      <Box
        component="pre"
        sx={{
          m: 0,
          mt: 0.5,
          p: 1,
          bgcolor: 'grey.900',
          color: 'grey.100',
          borderRadius: '4px',
          fontSize: '0.72rem',
          overflowX: 'auto',
          whiteSpace: 'pre-wrap',
          wordBreak: 'break-word',
        }}
      >
        {JSON.stringify(request.action.input, null, 2)}
      </Box>

      <Typography variant="caption" color="text.disabled" sx={{ display: 'block', mt: 0.5 }}>
        {request.action.inputHash?.slice(0, 19)}…
      </Typography>

      {expired ? (
        <Alert severity="warning" sx={{ mt: 1 }}>
          This approval expired. Conditions may have changed — the assistant must request it again.
        </Alert>
      ) : (
        <>
          <Collapse in={denying}>
            <TextField
              fullWidth
              size="small"
              label="Reason (optional)"
              value={reason}
              onChange={(event) => setReason(event.target.value)}
              sx={{ mt: 1 }}
            />
          </Collapse>

          {error && (
            <Alert severity="error" sx={{ mt: 1 }}>
              {error}
            </Alert>
          )}

          <Stack direction="row" spacing={1} sx={{ mt: 1 }}>
            <Button
              variant="contained"
              color="error"
              size="small"
              disabled={sending}
              onClick={() => respond('approve')}
            >
              Approve &amp; execute
            </Button>
            <Button
              variant="outlined"
              size="small"
              disabled={sending}
              onClick={() => (denying ? respond('deny') : setDenying(true))}
            >
              {denying ? 'Confirm deny' : 'Deny'}
            </Button>
          </Stack>
        </>
      )}
    </Box>
  );
};

/**
 * The human-in-the-loop gate for flight tools.
 *
 * Renders nothing when there is nothing parked, so it costs nothing in the
 * common case. Answers travel as structured responses keyed by `requestId` —
 * typing into the chat box never approves anything.
 */
const ToolApprovalGate = ({ chatId }) => {
  const dispatch = useDispatch();
  const requests = useSelector((state) => state.chat.pendingApprovals[chatId] ?? []);

  if (requests.length === 0) return null;

  const handleResponded = (requestId) =>
    dispatch(chatActions.resolveApprovals({ chatId, resolutions: [{ requestId }] }));

  return (
    <Box sx={{ px: 1, pt: 1, borderTop: '1px solid #e0e0e0', bgcolor: '#fff' }}>
      <Typography variant="overline" color="error" sx={{ fontWeight: 700 }}>
        Waiting for your approval
      </Typography>
      {requests.map((request) => (
        <ApprovalRequest
          key={request.requestId}
          request={request}
          chatId={chatId}
          onResponded={handleResponded}
        />
      ))}
    </Box>
  );
};

export default ToolApprovalGate;
