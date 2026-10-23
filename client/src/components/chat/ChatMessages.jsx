import { useState, use, memo } from 'react';
import { useDispatch } from 'react-redux';
import { Box, Button, ButtonBase, Collapse, Menu, MenuItem, Typography } from '@mui/material';
import { alpha } from '@mui/material/styles';
import AccountTreeOutlinedIcon from '@mui/icons-material/AccountTreeOutlined';
import CallSplitIcon from '@mui/icons-material/CallSplit';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import NorthEastIcon from '@mui/icons-material/NorthEast';
import PsychologyOutlinedIcon from '@mui/icons-material/PsychologyOutlined';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { forkConversation } from '../../store/chat';
import { ChatNavigationContext } from './ChatNavigationContext';
import StatusTag from './StatusTag';
import { CodeBlock, FormattedPayload } from './CodeBlock';
import { HEADING_FONT, kickerSx, MONO_FONT, OFFSCREEN_SKIP } from './chatTheme';
import { getMessageView } from '../../common/util/chatMessage';

const CodeRenderer = ({ inline, className, children, ...props }) => {
  const match = /language-(\w+)/.exec(className || '');
  if (!inline && match) {
    return <CodeBlock language={match[1]} value={String(children).replace(/\n$/, '')} {...props} />;
  }
  return (
    <code
      className={className}
      style={{
        backgroundColor: 'background.surface',
        padding: '1px 4px',
        fontFamily: MONO_FONT,
        fontSize: '0.9em',
      }}
      {...props}
    >
      {children}
    </code>
  );
};

const markdownComponents = { code: CodeRenderer };

// The assistant's text has no bubble: markdown set directly on the panel.
const markdownSx = {
  fontSize: 14,
  lineHeight: 1.5,
  overflowWrap: 'anywhere',
  '& > :first-of-type': { mt: 0 },
  '& > :last-child': { mb: 0 },
  '& p': { my: 0, mb: 1 },
  '& ul, & ol': { my: 0.5, pl: 2.75 },
  '& h1, & h2, & h3, & h4': { fontSize: 15, fontWeight: 600, my: 1 },
  '& a': { color: 'primary.700' },
  '& table': { borderCollapse: 'collapse', my: 1, fontSize: 12.5 },
  '& th, & td': { border: 1, borderColor: 'divider', px: 1, py: 0.5, textAlign: 'left' },
  '& th': { bgcolor: 'background.surface' },
  '& blockquote': {
    m: 0,
    my: 1,
    pl: 1.5,
    borderLeft: 2,
    borderLeftColor: 'primary.300',
    color: 'text.secondary',
  },
};

const MultipartContent = ({ content }) => (
  <Box>
    {content.map((block, i) => {
      if (block.type === 'input_text' && block.text?.trim()) {
        return (
          <Typography
            key={`text-${block.text}-${i}`}
            sx={{ fontSize: 14, lineHeight: 1.45, whiteSpace: 'pre-wrap', mb: 1 }}
          >
            {block.text}
          </Typography>
        );
      }
      if (block.type === 'input_image') {
        return (
          <Box
            key={`image-${block.image_url}`}
            component="img"
            src={block.image_url}
            alt="attached"
            sx={{ mt: 1, maxWidth: '100%', maxHeight: 300, display: 'block' }}
          />
        );
      }
      return null;
    })}
  </Box>
);

// A text row with a right-click "fork from here" menu. The estimate sizes it while it is skipped.
const TextMessage = ({ message, chatId, view }) => {
  const dispatch = useDispatch();
  const [contextMenu, setContextMenu] = useState(null);

  const isUser = view.role === 'user';
  const isError = view.status === 'error';

  const handleFork = async () => {
    setContextMenu(null);
    if (!chatId || !message.timestamp) return;
    try {
      await dispatch(forkConversation(chatId, message.timestamp));
    } catch (error) {
      console.error('Error forking conversation:', error);
    }
  };

  const body = isUser ? (
    <Box
      sx={{
        alignSelf: 'flex-end',
        maxWidth: '84%',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'flex-end',
        gap: 0.5,
      }}
    >
      <Typography sx={{ fontSize: 11, color: 'text.secondary' }}>
        {view.timestampLabel ? `You · ${view.timestampLabel}` : 'You'}
      </Typography>
      <Box
        sx={{
          bgcolor: 'primary.600',
          border: 1,
          borderColor: 'primary.600',
          color: 'primary.contrastText',
          borderRadius: '6px 6px 0 6px',
          px: 1.5,
          py: 1.125,
          fontSize: 14,
          lineHeight: 1.45,
          textWrap: 'pretty',
          overflowWrap: 'anywhere',
        }}
      >
        {view.type === 'multipart' ? (
          <MultipartContent content={view.content} />
        ) : (
          <Box sx={{ whiteSpace: 'pre-wrap' }}>{view.content}</Box>
        )}
      </Box>
    </Box>
  ) : (
    <Box
      sx={
        isError
          ? {
              border: '1px solid',
              borderColor: 'warning.main',
              px: 1.5,
              py: 1,
              color: 'warning.dark',
            }
          : undefined
      }
    >
      <Box sx={markdownSx}>
        <ReactMarkdown remarkPlugins={[remarkGfm]} components={markdownComponents}>
          {typeof view.content === 'string' ? view.content : JSON.stringify(view.content)}
        </ReactMarkdown>
      </Box>
      {view.timestampLabel && (
        <Typography sx={{ fontSize: 11, color: 'text.secondary', mt: 0.5 }}>
          {view.timestampLabel}
        </Typography>
      )}
    </Box>
  );

  return (
    <>
      <Box
        onContextMenu={(event) => {
          event.preventDefault();
          setContextMenu({ mouseX: event.clientX, mouseY: event.clientY });
        }}
        style={{ containIntrinsicSize: view.intrinsicSize }}
        sx={{ display: 'flex', flexDirection: 'column', ...OFFSCREEN_SKIP }}
      >
        {body}
      </Box>

      <Menu
        open={contextMenu !== null}
        onClose={() => setContextMenu(null)}
        anchorReference="anchorPosition"
        anchorPosition={
          contextMenu !== null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined
        }
      >
        <MenuItem onClick={handleFork} disabled={!chatId}>
          <CallSplitIcon fontSize="small" sx={{ mr: 1 }} />
          New chat from here
        </MenuItem>
      </Menu>
    </>
  );
};

const ReasoningRow = ({ content }) => {
  const [open, setOpen] = useState(false);
  return (
    <Box
      sx={{
        border: '1px solid',
        borderColor: 'divider',
        borderRadius: 1,
        overflow: 'hidden',
        bgcolor: 'background.paper',
        ...OFFSCREEN_SKIP,
        containIntrinsicSize: 'auto 34px',
      }}
    >
      <ButtonBase
        onClick={() => setOpen((value) => !value)}
        aria-expanded={open}
        sx={{
          width: '100%',
          display: 'flex',
          alignItems: 'center',
          gap: 1,
          px: 1.25,
          py: 0.875,
          justifyContent: 'flex-start',
          '&:hover': { bgcolor: (theme) => alpha(theme.palette.text.primary, 0.05) },
        }}
      >
        <PsychologyOutlinedIcon sx={{ fontSize: 14, color: 'primary.700' }} />
        <Typography sx={{ ...kickerSx, flex: 1, textAlign: 'left', letterSpacing: '0.1em' }}>
          Thinking
        </Typography>
        <ExpandMoreIcon
          sx={{
            fontSize: 14,
            transition: 'transform 120ms',
            transform: open ? 'rotate(180deg)' : 'none',
          }}
        />
      </ButtonBase>
      <Collapse in={open} unmountOnExit>
        <Box
          sx={{
            borderTop: '1px solid',
            borderColor: 'divider',
            px: 1.25,
            py: 1,
            fontFamily: MONO_FONT,
            fontSize: 12,
            lineHeight: 1.45,
            color: 'text.secondary',
            whiteSpace: 'pre-wrap',
          }}
        >
          {content}
        </Box>
      </Collapse>
    </Box>
  );
};

const SUBAGENT_DONE = new Set(['done', 'completed', 'success', 'succeeded']);
const SUBAGENT_FAILED = new Set(['error', 'incomplete', 'failed']);

const subagentTag = (status) => {
  if (SUBAGENT_FAILED.has(status)) return { tone: 'failed', label: 'Failed' };
  if (!status || SUBAGENT_DONE.has(status)) return { tone: 'done', label: 'Completed' };
  return { tone: 'running', label: status };
};

/** A subagent's result as the handoff's subagent card, with its full output one click away. */
const SubagentCard = ({ view }) => {
  const [open, setOpen] = useState(false);
  const { openChat } = use(ChatNavigationContext);
  const status = typeof view.content === 'object' ? view.content?.status : null;
  const description = typeof view.content === 'object' ? view.content?.description : null;
  const tag = subagentTag(status);
  const failed = tag.tone === 'error';

  return (
    <Box
      sx={{
        p: 1.5,
        display: 'flex',
        flexDirection: 'column',
        gap: 0.75,
        border: '1px solid',
        borderColor: 'divider',
        borderTop: 3,
        borderTopColor: failed ? 'error.main' : 'subagent.main',
        borderRadius: 1,
        bgcolor: failed ? 'error.tint' : 'subagent.tint',
      }}
    >
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
        <AccountTreeOutlinedIcon sx={{ fontSize: 14, color: 'primary.700' }} />
        <Typography sx={kickerSx}>Subagent</Typography>
        <Box sx={{ flex: 1 }} />
        <StatusTag tone={tag.tone} label={tag.label} />
      </Box>
      <Box sx={{ display: 'flex', alignItems: 'baseline', gap: 1, flexWrap: 'wrap' }}>
        <Typography
          sx={{ fontFamily: HEADING_FONT, fontWeight: 600, fontSize: 18, lineHeight: 1.2 }}
        >
          {view.agentName || view.name || 'Subagent'}
        </Typography>
        {view.name && view.agentName && (
          <Typography sx={{ fontSize: 12, color: 'text.secondary' }}>{view.name}</Typography>
        )}
      </Box>
      {description && (
        <Typography
          sx={{
            fontSize: 13,
            lineHeight: 1.45,
            color: 'neutral.800',
            textWrap: 'pretty',
          }}
        >
          {description}
        </Typography>
      )}
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mt: 0.5 }}>
        <Button size="small" onClick={() => setOpen((value) => !value)} sx={{ px: 0 }}>
          {open ? 'Hide result' : 'Show result'}
        </Button>
        <Box sx={{ flex: 1 }} />
        {view.subAgentChatId && openChat && (
          <Button
            size="small"
            variant="outlined"
            endIcon={<NorthEastIcon sx={{ fontSize: 14 }} />}
            onClick={() => openChat(view.subAgentChatId)}
            sx={{
              bgcolor: 'background.paper',
              color: failed ? 'error.main' : 'subagent.text',
              borderColor: failed ? 'error.border' : 'subagent.border',
            }}
          >
            Open chat
          </Button>
        )}
      </Box>
      <Collapse in={open} unmountOnExit>
        <FormattedPayload label="Result" content={view.content} />
      </Collapse>
    </Box>
  );
};

/** A transcript item that is not a tool call: text, reasoning, an image, a subagent's result. */
export const ChatMessage = memo(({ message, chatId }) => {
  const view = getMessageView(message);

  switch (view.type) {
    case 'reasoning':
      return <ReasoningRow content={view.content} />;
    case 'subagent_result':
      return <SubagentCard view={view} />;
    default:
      return <TextMessage message={message} chatId={chatId} view={view} />;
  }
});

/** Whether a message is the operator's own (it sits right, and does not carry the assistant label). */
export const isUserMessage = (message) => {
  const view = getMessageView(message);
  return view.role === 'user' && (view.type === 'text' || view.type === 'multipart');
};
