import { useState, useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { Box, Typography, Button, Stack, Chip, TextField, Collapse, Alert } from '@mui/material';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import HelpOutlineIcon from '@mui/icons-material/HelpOutlineOutlined';
import { chatActions } from '../../store/chat';
import { sendToolApprovalResponse } from '../../common/util/toolApproval';
import {
  formatCountdown,
  isFlightApproval,
  isTimed,
  requestTitle,
  secondsLeft,
  splitOptions,
} from '../../common/util/inputRequest';

/**
 * One request waiting for the operator.
 *
 * For a tool call it renders the input EXACTLY as it will execute — the operator is approving this
 * value, not the intent behind it, so nothing here is summarised or reformatted. The same card
 * serves both engines: the legacy one (with an expiry and an input hash) and eve (neither; its
 * options come with the request). A request that is not a tool call, such as a session limit,
 * shows its prompt and its options.
 */
const ApprovalRequest = ({ request, chatId, onResponded }) => {
  const timed = isTimed(request);
  const [remaining, setRemaining] = useState(() => (timed ? secondsLeft(request.expiresAt) : null));
  const [pendingNegative, setPendingNegative] = useState(null);
  const [reason, setReason] = useState('');
  const [answered, setAnswered] = useState(false);
  const [error, setError] = useState(null);
  const dispatch = useDispatch();
  // Why the server could not take the answer (it could not record or deliver it). While it is set
  // the card is usable again; it is cleared when the operator tries once more.
  const failure = useSelector((state) => state.chat.approvalFailures[request.requestId]);
  const sending = answered && !failure;

  useEffect(() => {
    if (!timed) return undefined;
    const tick = setInterval(() => setRemaining(secondsLeft(request.expiresAt)), 1000);
    return () => clearInterval(tick);
  }, [timed, request.expiresAt]);

  const expired = timed && remaining === 0;
  const flight = isFlightApproval(request);
  const { affirmative, negatives } = splitOptions(request);

  const respond = (optionId) => {
    setAnswered(true);
    setError(null);
    dispatch(
      chatActions.setApprovalFailures({
        failures: [{ requestId: request.requestId, message: null }],
      }),
    );
    try {
      sendToolApprovalResponse(chatId, [
        { requestId: request.requestId, optionId, text: reason || undefined },
      ]);
      // The legacy engine takes the answer synchronously, so its card goes at once. An eve card
      // stays until eve confirms (`resolved`): if the server cannot record or deliver the answer
      // it says so (`failure`), and a card that already vanished would leave the operator
      // believing a flight was approved when nothing was sent.
      if (request.source !== 'eve') onResponded?.(request.requestId);
    } catch (err) {
      setError(err.message);
      setAnswered(false);
    }
  };

  const handleNegative = (option) => {
    if (pendingNegative === option.id) respond(option.id);
    else setPendingNegative(option.id);
  };

  return (
    <Box
      sx={{
        border: '2px solid',
        borderColor: expired ? 'grey.400' : flight ? 'error.main' : 'primary.main',
        borderRadius: '8px',
        bgcolor: expired ? 'grey.50' : flight ? '#fff8f6' : 'grey.50',
        p: 1.5,
        mb: 1,
      }}
    >
      <Stack direction="row" alignItems="center" spacing={1} sx={{ mb: 1 }}>
        {flight ? (
          <WarningAmberIcon fontSize="small" color={expired ? 'disabled' : 'error'} />
        ) : (
          <HelpOutlineIcon fontSize="small" color={expired ? 'disabled' : 'primary'} />
        )}
        <Typography variant="subtitle2" sx={{ fontWeight: 700, flexGrow: 1 }}>
          {requestTitle(request)}
        </Typography>
        {timed && (
          <Chip
            size="small"
            label={expired ? 'Expired' : formatCountdown(remaining)}
            color={expired ? 'default' : 'error'}
            variant="outlined"
          />
        )}
      </Stack>

      {request.action?.input != null ? (
        <>
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
        </>
      ) : (
        request.prompt && <Typography variant="body2">{request.prompt}</Typography>
      )}

      {request.action?.inputHash && (
        <Typography variant="caption" color="text.disabled" sx={{ display: 'block', mt: 0.5 }}>
          {request.action.inputHash.slice(0, 19)}…
        </Typography>
      )}

      {expired ? (
        <Alert severity="warning" sx={{ mt: 1 }}>
          This approval expired. Conditions may have changed — the assistant must request it again.
        </Alert>
      ) : (
        <>
          <Collapse in={pendingNegative !== null}>
            <TextField
              fullWidth
              size="small"
              label="Reason (optional)"
              value={reason}
              onChange={(event) => setReason(event.target.value)}
              sx={{ mt: 1 }}
            />
          </Collapse>

          {(error || failure) && (
            <Alert severity="error" sx={{ mt: 1 }}>
              {error ?? failure}
            </Alert>
          )}

          <Stack direction="row" spacing={1} sx={{ mt: 1 }}>
            <Button
              variant="contained"
              color={flight ? 'error' : 'primary'}
              size="small"
              disabled={sending}
              onClick={() => respond(affirmative.id)}
            >
              {affirmative.label}
            </Button>
            {negatives.map((option) => (
              <Button
                key={option.id}
                variant="outlined"
                size="small"
                disabled={sending}
                onClick={() => handleNegative(option)}
              >
                {pendingNegative === option.id
                  ? `Confirm ${option.label.toLowerCase()}`
                  : option.label}
              </Button>
            ))}
          </Stack>
        </>
      )}
    </Box>
  );
};

/**
 * The human-in-the-loop gate for flight tools.
 *
 * Renders nothing when there is nothing waiting, so it costs nothing in the common case. Answers
 * travel as structured responses keyed by `requestId` — typing into the chat box never approves
 * anything (with eve it cancels the approval instead, which is the safe direction).
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
