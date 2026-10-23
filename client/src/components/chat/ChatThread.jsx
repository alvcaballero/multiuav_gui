import { Box, Button, CircularProgress, Typography } from '@mui/material';
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import ApprovalCard, { ResolvedApprovalRow } from './ApprovalCard';
import { ChatMessage, isUserMessage } from './ChatMessages';
import ToolCallRow from './ToolCallRow';
import { HEADING_FONT, kickerSx } from './chatTheme';
import { groupTranscript, mergeResolved } from '../../common/util/chatMessage';
import { activityText } from '../../common/util/inputRequest';

const SUGGESTIONS = [
  'Which drones are available?',
  'Inspect the entire wind farm',
  'Inspect the entire wind farm using all available UAVs to get a general assessment of its condition',
  'Use 2 UAVs to monitor the condition of the 4 wind turbines located in the north as soon as possible',
  'Inspect wind turbine B3 with a high level of detail',
  'Inspect the north face of the smaller building',
  'Quickly inspect lighting towers without entering the work area',
  'Inspect the crane tower in detail',
  'Inspecciona la línea A y la línea B de la granja de aerogeneradores.',
  'Inspecciona la turbina eólica B6 con un alto nivel de detalle.',
];

const EmptyThread = ({ showSuggestions, onSend }) => (
  <Box sx={{ flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column', gap: 2 }}>
    <Box
      sx={{
        flex: 1,
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        textAlign: 'center',
        gap: 1,
        py: 2,
      }}
    >
      <Box
        sx={{
          width: 80,
          height: 80,
          borderRadius: '50%',
          bgcolor: 'primary.100',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          mb: 2,
        }}
      >
        <PrecisionManufacturingIcon sx={{ fontSize: 40, color: 'primary.main' }} />
      </Box>
      <Typography
        sx={{
          fontFamily: HEADING_FONT,
          fontWeight: 600,
          fontSize: 20,
          letterSpacing: '0.1em',
          textTransform: 'uppercase',
        }}
      >
        Start a new conversation
      </Typography>
      <Typography sx={{ fontSize: 13.5, color: 'text.secondary', maxWidth: 320 }}>
        I can help you to create missions, control and monitor drones
      </Typography>
    </Box>
    {showSuggestions && (
      // A few show at a time: the rest scroll, so the welcome keeps the room.
      <Box
        sx={{
          maxHeight: 150,
          overflowY: 'auto',
          display: 'flex',
          flexDirection: 'column',
          gap: 0.75,
          flexShrink: 0,
        }}
      >
        {SUGGESTIONS.map((suggestion) => (
          <Button
            key={suggestion}
            variant="outlined"
            onClick={() => onSend(suggestion)}
            sx={{
              justifyContent: 'center',
              textAlign: 'center',
              py: 0.75,
              height: 'auto',
              flexShrink: 0,
              color: 'text.primary',
              fontWeight: 500,
            }}
          >
            {suggestion}
          </Button>
        ))}
      </Box>
    )}
  </Box>
);

/** "Task from Assistant": the first message of a subagent chat is the brief its parent gave it. */
const TaskCard = ({ message }) => {
  const content = message.message?.content;
  const text =
    typeof content === 'string' ? content : (content?.find?.((block) => block.text)?.text ?? '');
  return (
    <Box
      sx={{
        border: '1px solid',
        borderColor: 'divider',
        borderRadius: 1,
        bgcolor: 'background.surface',
        px: 1.5,
        py: 1.25,
        display: 'flex',
        flexDirection: 'column',
        gap: 0.5,
      }}
    >
      <Typography sx={kickerSx}>Task from Assistant</Typography>
      <Typography
        sx={{ fontSize: 13.5, lineHeight: 1.45, whiteSpace: 'pre-wrap', textWrap: 'pretty' }}
      >
        {text}
      </Typography>
    </Box>
  );
};

/** What the assistant is doing right now (the open turn). */
const WorkingRow = ({ text }) => (
  <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, py: 0.5 }}>
    <CircularProgress size={14} thickness={4} />
    <Typography sx={{ fontSize: 12.5, color: 'text.secondary' }}>{text}</Typography>
  </Box>
);

/**
 * Splits the transcript into the operator's messages and the assistant's runs. A run is everything
 * the assistant did between two messages of the operator — text, tool calls, decided approvals —
 * and carries the assistant label once.
 */
const toRuns = (items) => {
  const runs = [];
  for (const item of items) {
    if (item.kind === 'message' && isUserMessage(item.message)) {
      runs.push({ type: 'user', key: item.key, item });
    } else if (runs.at(-1)?.type === 'assistant') {
      runs.at(-1).items.push(item);
    } else {
      runs.push({ type: 'assistant', key: item.key, items: [item] });
    }
  }
  return runs;
};

/**
 * The scrolling content of the panel: older-messages marker, the transcript, the approvals waiting
 * on the operator and the activity of the open turn. The scroller itself, and the refs that keep it
 * pinned to the newest message, belong to `ChatPanel`.
 */
const ChatThread = ({
  chatId,
  messages,
  pendingApprovals,
  resolvedApprovals,
  isSubagent,
  assistantLabel,
  working,
  activity,
  hasMoreOlder,
  loadingOlderMessages,
  showSuggestions,
  onSend,
}) => {
  const items = mergeResolved(groupTranscript(messages), resolvedApprovals);
  const runs = toRuns(items);
  const workingText = working ? (activityText(activity) ?? 'Thinking…') : null;

  // The assistant's side of the thread ends with what is waiting on the operator and what is
  // running now. If the operator spoke last, that opens a run of its own.
  const hasTail = pendingApprovals.length > 0 || workingText;
  if (hasTail && runs.at(-1)?.type !== 'assistant') {
    runs.push({ type: 'assistant', key: 'tail', items: [] });
  }
  const lastRun = runs.at(-1);

  // The last tool call without a result is the one in flight, while the turn is open.
  const lastTool = [...items].reverse().find((item) => item.kind === 'tool');
  const runningKey = working && lastTool && !lastTool.output ? lastTool.key : null;

  if (items.length === 0 && !hasTail) {
    return <EmptyThread showSuggestions={showSuggestions} onSend={onSend} />;
  }

  return (
    <>
      {(loadingOlderMessages || hasMoreOlder) && (
        <Box sx={{ display: 'flex', justifyContent: 'center' }}>
          {loadingOlderMessages ? (
            <CircularProgress size={16} />
          ) : (
            <Typography sx={{ fontSize: 11, color: 'text.secondary' }}>
              Scroll up to load earlier messages
            </Typography>
          )}
        </Box>
      )}

      {runs.map((run, index) => {
        if (run.type === 'user') {
          return isSubagent && index === 0 ? (
            <TaskCard key={run.key} message={run.item.message} />
          ) : (
            <ChatMessage key={run.key} message={run.item.message} chatId={chatId} />
          );
        }

        const isLast = run === lastRun;
        return (
          <Box key={run.key} sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
            <Typography
              sx={{
                fontFamily: HEADING_FONT,
                fontWeight: 600,
                fontSize: 12,
                letterSpacing: '0.1em',
                textTransform: 'uppercase',
                color: 'primary.700',
              }}
            >
              {assistantLabel}
            </Typography>
            {run.items.map((item) => {
              if (item.kind === 'tool') {
                return (
                  <ToolCallRow
                    key={item.key}
                    call={item.call}
                    output={item.output}
                    running={item.key === runningKey}
                  />
                );
              }
              if (item.kind === 'resolved') {
                return <ResolvedApprovalRow key={item.key} resolved={item.resolved} />;
              }
              return <ChatMessage key={item.key} message={item.message} chatId={chatId} />;
            })}
            {isLast &&
              pendingApprovals.map((request) => (
                <ApprovalCard key={request.requestId} request={request} chatId={chatId} />
              ))}
            {isLast && workingText && <WorkingRow text={workingText} />}
          </Box>
        );
      })}
    </>
  );
};

export default ChatThread;
