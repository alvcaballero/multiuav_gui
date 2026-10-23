import { useState, useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { Box, Button, TextField, Typography } from '@mui/material';
import CheckIcon from '@mui/icons-material/Check';
import CloseIcon from '@mui/icons-material/Close';
import GppMaybeOutlinedIcon from '@mui/icons-material/GppMaybeOutlined';
import SubdirectoryArrowRightIcon from '@mui/icons-material/SubdirectoryArrowRight';
import { chatActions } from '../../store/chat';
import { sendToolApprovalResponse } from '../../common/util/toolApproval';
import { paramRows, formatTimestamp } from '../../common/util/chatMessage';
import {
  formatCountdown,
  isFlightApproval,
  isTimed,
  requestTitle,
  secondsLeft,
  splitOptions,
} from '../../common/util/inputRequest';
import StatusTag from './StatusTag';
import { MONO_FONT } from './chatTheme';

/**
 * One request waiting for the operator, as the handoff's approval card.
 *
 * For a tool call it lists the input EXACTLY as it will execute — the operator is approving this
 * value, not the intent behind it, so nothing here is summarised or cut. The same card serves both
 * engines: the legacy one (with an expiry and an input hash) and eve (neither; its options come
 * with the request). A request that is not a tool call, such as a session limit, shows its prompt
 * and its own options.
 *
 * "Do this instead…" is a rejection that carries the operator's instruction as its text: the
 * command is never sent, and the assistant gets what to do in its place.
 */
const rejectSx = {
  bgcolor: 'background.paper',
  // error.text, not error.main: it must still read on the hover tint.
  color: 'error.text',
  borderColor: 'error.border',
  '&:hover': { bgcolor: 'error.tint', borderColor: 'error.border' },
};

const ApprovalCard = ({ request, chatId }) => {
  const dispatch = useDispatch();
  const timed = isTimed(request);
  const [remaining, setRemaining] = useState(() => (timed ? secondsLeft(request.expiresAt) : null));
  const [instead, setInstead] = useState(false);
  const [instruction, setInstruction] = useState('');
  const [answered, setAnswered] = useState(false);
  const [error, setError] = useState(null);
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
  const isTool = isFlightApproval(request);
  const { affirmative, negatives } = splitOptions(request);
  const [reject, ...otherNegatives] = negatives;
  const rows = isTool ? paramRows(request.action?.input) : [];

  const respond = (optionId, text) => {
    setAnswered(true);
    setError(null);
    dispatch(
      chatActions.setApprovalFailures({
        failures: [{ requestId: request.requestId, message: null }],
      }),
    );
    dispatch(chatActions.noteApprovalAnswer({ requestId: request.requestId, optionId, text }));
    try {
      sendToolApprovalResponse(chatId, [
        { requestId: request.requestId, optionId, text: text || undefined },
      ]);
      // The legacy engine takes the answer synchronously, so its card goes at once. An eve card
      // stays until eve confirms (`resolved`): if the server cannot record or deliver the answer
      // it says so (`failure`), and a card that already vanished would leave the operator
      // believing a flight was approved when nothing was sent.
      if (request.source !== 'eve') {
        dispatch(
          chatActions.resolveApprovals({ chatId, resolutions: [{ requestId: request.requestId }] }),
        );
      }
    } catch (err) {
      setError(err.message);
      setAnswered(false);
    }
  };

  const sendInstead = () => respond(reject.id, instruction.trim());

  return (
    <Box
      sx={{
        p: 1.5,
        display: 'flex',
        flexDirection: 'column',
        gap: 1.25,
        border: '1px solid',
        borderColor: expired ? 'divider' : 'warning.main',
        bgcolor: expired ? 'transparent' : 'warning.tint',
        borderRadius: 1,
      }}
    >
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
        <GppMaybeOutlinedIcon sx={{ fontSize: 16, color: 'warning.icon' }} />
        <Typography
          sx={{
            fontWeight: 600,
            fontSize: 13,
            letterSpacing: '0.12em',
            textTransform: 'uppercase',
            color: 'warning.text',
            flex: 1,
          }}
        >
          {isTool ? 'Approval required' : 'Waiting for your input'}
        </Typography>
        {expired ? (
          <StatusTag label="Expired" />
        ) : (
          <StatusTag
            tone="attention"
            label={timed ? `Waiting · ${formatCountdown(remaining)}` : 'Waiting'}
          />
        )}
      </Box>

      {isTool ? (
        <>
          <Typography sx={{ fontFamily: MONO_FONT, fontSize: 14, overflowWrap: 'anywhere' }}>
            {requestTitle(request)}
          </Typography>
          {rows.length > 0 && (
            <Box
              sx={{
                display: 'grid',
                gridTemplateColumns: 'auto 1fr',
                gap: '4px 16px',
                fontSize: 13,
              }}
            >
              {rows.map(({ key, value }) => (
                <Box key={key} sx={{ display: 'contents' }}>
                  <Typography
                    component="span"
                    sx={{ color: 'text.secondary', fontSize: 'inherit' }}
                  >
                    {key}
                  </Typography>
                  <Typography
                    component="span"
                    sx={{ fontSize: 'inherit', overflowWrap: 'anywhere', whiteSpace: 'pre-wrap' }}
                  >
                    {value}
                  </Typography>
                </Box>
              ))}
            </Box>
          )}
        </>
      ) : (
        request.prompt && <Typography sx={{ fontSize: 14 }}>{request.prompt}</Typography>
      )}

      {request.action?.inputHash && (
        <Typography sx={{ fontSize: 11, color: 'text.secondary', fontFamily: MONO_FONT }}>
          {request.action.inputHash.slice(0, 19)}…
        </Typography>
      )}

      {(error || failure) && (
        <Typography role="alert" sx={{ fontSize: 13, color: 'error.main' }}>
          {error ?? failure}
        </Typography>
      )}

      {expired ? (
        <Typography sx={{ fontSize: 13, color: 'text.secondary' }}>
          This approval expired. Conditions may have changed — the assistant must request it again.
        </Typography>
      ) : instead ? (
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 0.75 }}>
          <Typography
            component="label"
            htmlFor={`instead-${request.requestId}`}
            sx={{ fontSize: 12, color: 'text.secondary' }}
          >
            Tell the assistant what to do instead
          </Typography>
          <TextField
            id={`instead-${request.requestId}`}
            multiline
            minRows={2}
            maxRows={6}
            fullWidth
            autoFocus
            placeholder="e.g. Start from WP3 and fly at 30 m"
            value={instruction}
            onChange={(event) => setInstruction(event.target.value)}
            slotProps={{ input: { sx: { fontSize: 13 } } }}
          />
          <Box sx={{ display: 'flex', gap: 1, justifyContent: 'flex-end' }}>
            <Button size="small" onClick={() => setInstead(false)} disabled={sending}>
              Cancel
            </Button>
            <Button
              size="small"
              variant="contained"
              disabled={sending || !instruction.trim()}
              onClick={sendInstead}
            >
              Send instead
            </Button>
          </Box>
        </Box>
      ) : (
        <Box sx={{ display: 'flex', gap: 1, alignItems: 'center', flexWrap: 'wrap' }}>
          <Button
            variant="contained"
            color="success"
            disabled={sending}
            onClick={() => respond(affirmative.id)}
            startIcon={<CheckIcon sx={{ fontSize: 14 }} />}
          >
            {isTool ? 'Approve' : affirmative.label}
          </Button>
          {reject && (
            <Button
              variant="outlined"
              disabled={sending}
              onClick={() => respond(reject.id)}
              sx={rejectSx}
            >
              {isTool ? 'Reject' : reject.label}
            </Button>
          )}
          {otherNegatives.map((option) => (
            <Button
              key={option.id}
              variant="outlined"
              disabled={sending}
              onClick={() => respond(option.id)}
              sx={rejectSx}
            >
              {option.label}
            </Button>
          ))}
          {isTool && reject && (
            <Button
              disabled={sending}
              onClick={() => setInstead(true)}
              startIcon={<SubdirectoryArrowRightIcon sx={{ fontSize: 14 }} />}
            >
              Do this instead…
            </Button>
          )}
        </Box>
      )}
    </Box>
  );
};

const RESOLVED_TEXT = {
  approved: 'approved by you',
  rejected: 'rejected by you · not sent',
  redirected: 'replaced by your instruction',
  expired: 'expired · not sent',
  invalid: 'not accepted · not sent',
};

// Green when it went through, red when it was refused, amber for what changed course or did not take.
const RESOLVED_LOOK = {
  approved: { border: 'success.border', bg: 'success.tint' },
  rejected: { border: 'error.rowBorder', bg: 'error.tint' },
  default: { border: 'warning.border', bg: 'warning.tint' },
};

/** The line left in the thread once a request was decided (the card is replaced by it). */
export const ResolvedApprovalRow = ({ resolved }) => {
  const look = RESOLVED_LOOK[resolved.outcome] ?? RESOLVED_LOOK.default;
  return (
    <Box
      sx={{
        display: 'flex',
        alignItems: 'baseline',
        gap: 1,
        flexWrap: 'wrap',
        px: 1.25,
        py: 0.75,
        border: '1px solid',
        borderColor: look.border,
        bgcolor: look.bg,
        borderRadius: 1,
        fontSize: 12.5,
        color: 'grey.800',
      }}
    >
      <Box component="span" sx={{ alignSelf: 'center', display: 'flex' }}>
        {resolved.outcome === 'approved' ? (
          <CheckIcon sx={{ fontSize: 14, color: 'success.main' }} />
        ) : resolved.outcome === 'rejected' ? (
          <CloseIcon sx={{ fontSize: 14, color: 'error.main' }} />
        ) : (
          <SubdirectoryArrowRightIcon sx={{ fontSize: 14, color: 'grey.700' }} />
        )}
      </Box>
      <Typography component="span" sx={{ fontFamily: MONO_FONT, fontSize: 12 }}>
        {resolved.title}
      </Typography>
      <Typography component="span" sx={{ flex: 1, fontSize: 'inherit' }}>
        {RESOLVED_TEXT[resolved.outcome] ?? resolved.outcome}
      </Typography>
      <Typography component="span" sx={{ fontSize: 11, color: 'text.secondary' }}>
        {formatTimestamp(resolved.at)}
      </Typography>
      {resolved.instruction && (
        <Typography sx={{ flexBasis: '100%', fontSize: 13, pl: 2.75, overflowWrap: 'anywhere' }}>
          “{resolved.instruction}”
        </Typography>
      )}
    </Box>
  );
};

export default ApprovalCard;
