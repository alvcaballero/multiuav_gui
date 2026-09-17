import { useState, memo } from 'react';
import { useDispatch } from 'react-redux';
import {
  ListItem,
  Typography,
  Box,
  Button,
  Avatar,
  Stack,
  IconButton,
  Menu,
  MenuItem as MuiMenuItem,
} from '@mui/material';
import PersonIcon from '@mui/icons-material/Person';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ContentCopyIcon from '@mui/icons-material/ContentCopy';
import CheckIcon from '@mui/icons-material/Check';
import CallSplitIcon from '@mui/icons-material/CallSplit';
import Tooltip from '@mui/material/Tooltip';
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { PrismLight as SyntaxHighlighter } from 'react-syntax-highlighter';
import json from 'react-syntax-highlighter/dist/esm/languages/prism/json';
import yaml from 'react-syntax-highlighter/dist/esm/languages/prism/yaml';
import bash from 'react-syntax-highlighter/dist/esm/languages/prism/bash';
import python from 'react-syntax-highlighter/dist/esm/languages/prism/python';
import javascript from 'react-syntax-highlighter/dist/esm/languages/prism/javascript';
import markdown from 'react-syntax-highlighter/dist/esm/languages/prism/markdown';
import vscDarkPlus from 'react-syntax-highlighter/dist/esm/styles/prism/vsc-dark-plus';
import { forkConversation } from '../../store/chat';

// PrismLight ships no grammars: each one is opted into explicitly. The full
// `Prism` build pulls refractor's ~594 languages and the 45-theme style barrel
// into the bundle, to render what is in practice almost entirely JSON (tool
// calls and tool results). Register only what the transcript can realistically
// contain — an unregistered language degrades to plain text, it does not throw.
SyntaxHighlighter.registerLanguage('json', json);
SyntaxHighlighter.registerLanguage('yaml', yaml);
SyntaxHighlighter.registerLanguage('bash', bash);
SyntaxHighlighter.registerLanguage('python', python);
SyntaxHighlighter.registerLanguage('javascript', javascript);
SyntaxHighlighter.registerLanguage('markdown', markdown);

// --- Data transformation utils ---

const convertMsg = (msg) => {
  if (!msg || !msg.message)
    return { role: 'assistant', type: 'error', content: 'Invalid message', status: 'error' };

  const status = msg.message.status || null;

  // Multipart content: array of input_text / input_image blocks (user messages with images)
  if (
    Array.isArray(msg.message.content) &&
    msg.message.content.some((b) => b.type === 'input_image')
  ) {
    return {
      role: msg.message.role,
      type: 'multipart',
      content: msg.message.content,
      status,
    };
  }

  // Orchestrator-authored nudge (e.g. retry after a malformed tool call),
  // not from the user or a subagent — never shown in the transcript.
  if (msg.message.type === 'system_directive') {
    return { role: 'system', type: 'system_directive', content: msg.message.content };
  }

  if (
    typeof msg.message.content === 'string' &&
    (msg.message.type === 'text' || !msg.message.type)
  ) {
    return {
      role: msg.message.role,
      content: msg.message.content,
      type: 'text',
      status,
    };
  }

  if (msg.message.type === 'reasoning') {
    return { role: 'assistant', type: 'reasoning', content: msg.message.content };
  }

  if (msg.message.type === 'message') {
    let contentText = '';
    if (typeof msg.message.content === 'string') {
      contentText = msg.message.content;
    } else if (Array.isArray(msg.message.content)) {
      const textPart = msg.message.content.find((c) => c.type === 'text');
      contentText = textPart ? textPart.text : msg.message.content[0]?.text || '';
    }
    return { role: msg.message.role, type: 'text', content: contentText };
  }

  if (msg.message.type === 'function_call' || msg.message.type === 'tool_call') {
    let args = msg.message.arguments || msg.message.content;
    if (typeof args === 'string') {
      try {
        args = JSON.parse(args);
      } catch {
        // Keep as string if not valid JSON
      }
    }

    return {
      role: 'assistant',
      type: 'function_call',
      content: args,
      name: msg.message.name,
      call_id: msg.message.call_id,
    };
  }

  // Async subagent result appended to the chat. Rendered as its own kind so it
  // never shows up as a user bubble — it arrives in the user turn, but the user
  // did not write it.
  if (msg.message.type === 'subagent_result') {
    let output = msg.message.output;

    if (typeof output === 'string') {
      try {
        output = JSON.parse(output);
      } catch {
        /* keep as string */
      }
    }

    return {
      role: 'assistant',
      type: 'subagent_result',
      content: output,
      name: msg.message.name,
      agentName: msg.message.agentName,
      subAgentChatId: msg.message.subAgentChatId,
      status: output?.status ?? status,
    };
  }

  if (msg.message.type === 'function_call_output') {
    let output = msg.message.output || msg.message.content;

    if (typeof output === 'string') {
      try {
        output = JSON.parse(output);
      } catch {
        /* keep as string */
      }
    }

    // MCP wraps the real payload inside content[0].text as a JSON string — unwrap it
    if (output?.content?.[0]?.text) {
      try {
        output = JSON.parse(output.content[0].text);
      } catch {
        /* keep outer */
      }
    }

    return {
      role: 'assistant',
      type: 'function_call_output',
      content: output,
      name: msg.message.name,
      call_id: msg.message.call_id,
    };
  }

  // Default fallback
  return {
    role: msg.message.role,
    type: 'text',
    content:
      typeof msg.message.content === 'string'
        ? msg.message.content
        : JSON.stringify(msg.message.content),
    status,
  };
};

const formatMcpContent = (content) => {
  if (typeof content === 'string') {
    return content;
  }

  if (typeof content !== 'object' || content === null) {
    return JSON.stringify(content, null, 2);
  }

  if (content.content && Array.isArray(content.content)) {
    const textParts = content.content.flatMap((item) => {
      if (item.type !== 'text' || !item.text) return [];
      let text = item.text;
      if (typeof text === 'string') {
        text = text
          .replace(/\\n/g, '\n')
          .replace(/\\t/g, '\t')
          .replace(/\\"/g, '"')
          .replace(/\\\\/g, '\\');
      }
      return [text];
    });

    if (textParts.length > 0) {
      return textParts.join('\n');
    }
  }

  if (content.text && typeof content.text === 'string') {
    return content.text
      .replace(/\\n/g, '\n')
      .replace(/\\t/g, '\t')
      .replace(/\\"/g, '"')
      .replace(/\\\\/g, '\\');
  }

  return JSON.stringify(content, null, 2);
};

const detectContentLanguage = (content) => {
  if (typeof content === 'string') {
    const trimmed = content.trim();
    if (
      (trimmed.startsWith('{') && trimmed.endsWith('}')) ||
      (trimmed.startsWith('[') && trimmed.endsWith(']'))
    ) {
      return 'json';
    }
    return 'text';
  }
  return 'json';
};

const getResultSummary = (content) => {
  if (typeof content !== 'object' || content === null) {
    return { summary: 'Result', hasError: false };
  }

  const toolResult = content?.content?.[0]?.text;

  if (content.error) return { summary: 'Error', hasError: true };
  if (toolResult && toolResult.includes('MCP error')) return { summary: 'Result', hasError: true };
  if (content.success === false) return { summary: 'Fault', hasError: true };
  if (content.mission || content.missionId) return { summary: 'Created Mission', hasError: false };
  if (content.devices || Array.isArray(content))
    return { summary: 'Information getted successfully', hasError: false };
  return { summary: 'Result', hasError: false };
};

const formatTimestamp = (ts) => {
  if (!ts) return null;
  const date = new Date(ts);
  if (isNaN(date.getTime())) return null;
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
};

// --- Derivation cache ---

/**
 * Memoises a pure unary derivation on the IDENTITY of its object argument.
 *
 * Every function above turns a message payload into something renderable, and
 * all of them are expensive for what they are: `convertMsg` can run three nested
 * `JSON.parse` passes, `formatMcpContent` runs four chained regex replacements
 * over strings that reach ~145 KB in this codebase's own transcripts, and both
 * finish with a `JSON.stringify(..., null, 2)`. React calls them again on every
 * render, on payloads that never change — a message, once stored, is immutable.
 *
 * A WeakMap keyed on the payload object is the exact shape of that invariant:
 * the derived value is valid for as long as the object it came from is alive,
 * and it is collected with it, so switching chats or trimming a conversation to
 * MAX_LIVE_MESSAGES cannot grow this cache without bound. Keying on a message id
 * would not work here — the server sends no stable id, and the client-side
 * fallback differs between a live message and the same message after a reload.
 *
 * Non-object arguments (a tool output that failed to parse stays a string) are
 * passed straight through: they cannot key a WeakMap, and for them these
 * derivations are already O(1).
 */
const memoizeByIdentity = (fn) => {
  const cache = new WeakMap();
  return (arg) => {
    if (arg === null || typeof arg !== 'object') return fn(arg);
    if (cache.has(arg)) return cache.get(arg);
    const value = fn(arg);
    cache.set(arg, value);
    return value;
  };
};

/**
 * Placeholder height for a chat bubble the browser has not rendered yet.
 *
 * A bubble's height varies wildly with its text — across this codebase's own
 * transcripts a text message runs 580 chars at the median, 4.1k at p75 and 39k
 * at p99, so a single constant is wrong by more than an order of magnitude for
 * most of them, and `contain-intrinsic-size` being wrong is exactly what makes a
 * scrollbar jump. Counting wrapped lines gets within the right ballpark, which is
 * all that is needed: once a row has been rendered the `auto` keyword makes the
 * browser reuse its real height and the estimate stops mattering.
 */
const BUBBLE_CHARS_PER_LINE = 52; // ~400px of text column at body1 size
const BUBBLE_LINE_HEIGHT = 24;
const BUBBLE_CHROME = 56; // padding, timestamp row, row margin
const BUBBLE_IMAGE_HEIGHT = 300; // matches the maxHeight on attached images
const MAX_BUBBLE_ESTIMATE = 6000;

const countWrappedLines = (text) =>
  text
    .split('\n')
    .reduce(
      (total, line) => total + Math.max(1, Math.ceil(line.length / BUBBLE_CHARS_PER_LINE)),
      0,
    );

const estimateBubbleHeight = (content) => {
  let lines = 0;
  let images = 0;

  if (typeof content === 'string') {
    lines = countWrappedLines(content);
  } else if (Array.isArray(content)) {
    for (const block of content) {
      if (block?.type === 'input_image') images += 1;
      else if (typeof block?.text === 'string') lines += countWrappedLines(block.text);
    }
  } else if (content != null) {
    lines = countWrappedLines(String(content));
  }

  const px = lines * BUBBLE_LINE_HEIGHT + images * BUBBLE_IMAGE_HEIGHT + BUBBLE_CHROME;
  // Bucketed so near-identical messages share one placeholder value.
  return Math.min(Math.ceil(px / 50) * 50, MAX_BUBBLE_ESTIMATE);
};

/**
 * Kinds that render as an accordion row. Collapsed, all of them are the same
 * height, so `accordionRowSx` carries one shared estimate and they need none of
 * their own. Everything else — including `convertMsg`'s fallbacks for an invalid
 * or unrecognised payload — falls through to a bubble, and a bubble under
 * `content-visibility` MUST get a size or it collapses to nothing off screen.
 * Mirrors the switch in MessageBubble; listing the closed set is what keeps a new
 * fallthrough kind from silently shipping without one.
 */
const ACCORDION_TYPES = new Set([
  'system_directive',
  'reasoning',
  'function_call',
  'subagent_result',
  'function_call_output',
]);

/** Full view model of a raw message: protocol conversion, timestamp, size hint. */
const getMessageView = memoizeByIdentity((msg) => {
  const converted = convertMsg(msg);
  return {
    ...converted,
    timestampLabel: formatTimestamp(msg?.timestamp),
    intrinsicSize: ACCORDION_TYPES.has(converted.type)
      ? null
      : `auto ${estimateBubbleHeight(converted.content)}px`,
  };
});

/**
 * Display form of a tool/subagent payload. Language detection is folded in
 * because it inspects the FORMATTED string — computing it here avoids a
 * `.trim()` copy of a six-figure-byte string on every render.
 */
const getFormattedResult = memoizeByIdentity((content) => {
  const text = formatMcpContent(content);
  return { text, language: detectContentLanguage(text) };
});

const getCachedResultSummary = memoizeByIdentity(getResultSummary);

const getPrettyJson = memoizeByIdentity((content) =>
  typeof content === 'object' ? JSON.stringify(content, null, 2) : content,
);

// --- Style constants ---

const COLORS = {
  neutral: '#9e9e9e',
  info: '#2196f3',
  infoText: '#1976d2',
  success: '#4caf50',
  successText: '#388e3c',
  error: '#f44336',
  errorText: '#d32f2f',
  purple: '#7b1fa2',
  purpleText: '#6a1b9a',
};

const accordionStyle = (borderColor) => ({
  boxShadow: 'none',
  bgcolor: 'transparent',
  border: 'none',
  borderLeft: `4px solid ${borderColor}`,
  '&:before': { display: 'none' },
  margin: '4px 0',
});

const summaryStyle = {
  minHeight: '36px',
  height: '36px',
  padding: '0 8px',
  '& .MuiAccordionSummary-content': { margin: 0 },
};

// --- Presentation primitives ---

/**
 * Characters handed to the highlighter before a payload is cut off behind an
 * explicit "show everything" click.
 *
 * Tool results in this codebase's own transcripts average ~2.8 KB and peak near
 * 146 KB — thousands of lines of JSON that no one reads in a <pre>; they get
 * skimmed, then copied. Highlighting all of it turns every token into a DOM
 * node, and since a collapsed accordion now unmounts its body, that cost is paid
 * again on every open. Capping it bounds the worst case by design instead of by
 * luck, while the copy button still puts the FULL payload on the clipboard.
 */
const MAX_HIGHLIGHT_CHARS = 2000;

/** Cuts at the last line break inside the budget, so the preview ends on a whole line. */
const truncateToLine = (text) => {
  const slice = text.slice(0, MAX_HIGHLIGHT_CHARS);
  const lastBreak = slice.lastIndexOf('\n');
  return lastBreak > 0 ? slice.slice(0, lastBreak) : slice;
};

const codeBlockHeaderSx = {
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
  bgcolor: '#1e1e1e',
  color: '#e0e0e0',
  px: 2,
  py: 0.5,
  fontSize: '0.75rem',
  borderBottom: '1px solid #333',
};

const codeBlockFooterSx = {
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
  gap: 1,
  bgcolor: '#1e1e1e',
  color: '#9e9e9e',
  px: 2,
  py: 0.5,
  borderTop: '1px solid #333',
};

const CodeBlock = ({ language, value }) => {
  const [copied, setCopied] = useState(false);
  const [showFull, setShowFull] = useState(false);

  const text = typeof value === 'string' ? value : String(value ?? '');
  const isTruncated = !showFull && text.length > MAX_HIGHLIGHT_CHARS;
  const shown = isTruncated ? truncateToLine(text) : text;

  const handleCopy = () => {
    // Always the full payload, never what happens to be on screen.
    navigator.clipboard.writeText(text);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <Box sx={{ position: 'relative', borderRadius: 1, overflow: 'hidden', my: 1 }}>
      <Box sx={codeBlockHeaderSx}>
        <Typography variant="caption" sx={{ fontFamily: 'monospace' }}>
          {language || 'text'}
        </Typography>
        <Tooltip title={copied ? 'Copiado!' : 'Copiar todo'}>
          <IconButton onClick={handleCopy} size="small" sx={{ color: '#e0e0e0', p: 0.5 }}>
            {copied ? <CheckIcon fontSize="inherit" /> : <ContentCopyIcon fontSize="inherit" />}
          </IconButton>
        </Tooltip>
      </Box>
      <SyntaxHighlighter
        language={language || 'text'}
        style={vscDarkPlus}
        customStyle={{ margin: 0, borderRadius: 0, fontSize: '0.85rem' }}
      >
        {shown}
      </SyntaxHighlighter>
      {isTruncated && (
        <Box sx={codeBlockFooterSx}>
          <Typography variant="caption" sx={{ fontFamily: 'monospace' }}>
            {`${shown.length.toLocaleString()} de ${text.length.toLocaleString()} caracteres`}
          </Typography>
          <Button
            size="small"
            onClick={() => setShowFull(true)}
            sx={{ color: '#90caf9', textTransform: 'none', minWidth: 0, py: 0 }}
          >
            Mostrar todo
          </Button>
        </Box>
      )}
    </Box>
  );
};

const FormattedResult = ({ content }) => {
  const { text, language } = getFormattedResult(content);
  return <CodeBlock language={language} value={text} />;
};

/**
 * Off-screen transcript rows are skipped by the browser instead of laid out,
 * styled and painted. `content-visibility: auto` is what buys that; the paired
 * `contain-intrinsic-size` is what makes it safe, because a skipped row would
 * otherwise collapse to zero height and wreck the scrollbar. Its `auto` keyword
 * means the placeholder size is only a guess until the row has been rendered
 * once — after that the browser reuses the real height it remembers, so scroll
 * position stays put when a row leaves the viewport.
 *
 * Preferred over windowing here because the rows stay in the DOM: find-in-page,
 * selecting and copying a whole conversation, and the existing scroll-anchoring
 * in `loadOlderMessages` all keep working untouched.
 */
const OFFSCREEN_SKIP = { contentVisibility: 'auto' };

// A message row that hosts a single accordion (reasoning / tool call / tool result).
// Collapsed, one is the 36px summary plus its margins.
const accordionRowSx = {
  mb: 1,
  display: 'block',
  px: 2,
  ...OFFSCREEN_SKIP,
  containIntrinsicSize: 'auto 68px',
};

const MessageListItem = ({ children }) => <ListItem sx={accordionRowSx}>{children}</ListItem>;

// Chat bubble rows. Two frozen variants rather than one object built per render:
// the side a bubble sits on is the only thing that varies, and an inline `sx`
// literal makes Emotion re-serialise the whole style object on every render.
// `contain-intrinsic-size` is NOT set here — it is per message, applied inline
// from `intrinsicSize` on the view model.
const textRowBaseSx = {
  mb: 2,
  alignItems: 'flex-start',
  cursor: 'context-menu',
  ...OFFSCREEN_SKIP,
};
const textRowAiSx = { ...textRowBaseSx, justifyContent: 'flex-start' };
const textRowUserSx = { ...textRowBaseSx, justifyContent: 'flex-end' };

// MUI's Accordion wraps its children in a <Collapse> that keeps them MOUNTED
// when closed — it only animates height to 0. Every collapsed tool call and tool
// result would therefore still run its payload through the syntax highlighter and
// materialise the resulting token spans in the DOM. Tool payloads here average
// ~3-4.5 KB and peak near 145 KB, so a long transcript builds tens of thousands
// of invisible nodes. Unmounting on exit makes a closed accordion cost nothing.
const collapseSlotProps = { transition: { unmountOnExit: true } };

const AccordionBlock = ({ borderColor, header, detailsSx, children }) => (
  <Accordion sx={accordionStyle(borderColor)} slotProps={collapseSlotProps}>
    <AccordionSummary expandIcon={<ExpandMoreIcon fontSize="small" />} sx={summaryStyle}>
      {header}
    </AccordionSummary>
    <AccordionDetails sx={{ pt: 1, pb: 1, pl: 2, pr: 1, ...detailsSx }}>
      {children}
    </AccordionDetails>
  </Accordion>
);

// Shared header row: colored label, optional action (extra), timestamp pinned right.
const AccordionHeader = ({ label, labelColor, extra, timestampLabel }) => (
  <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, width: '100%' }}>
    <Typography
      variant="caption"
      sx={{ color: labelColor, fontWeight: 'bold', fontFamily: 'monospace', fontSize: '0.85rem' }}
    >
      {label}
    </Typography>
    {extra}
    {timestampLabel && (
      <Typography
        variant="caption"
        sx={{ color: 'text.disabled', fontSize: '0.65rem', ml: extra ? 1 : 'auto' }}
      >
        {timestampLabel}
      </Typography>
    )}
  </Box>
);

export const WelcomeMessage = () => {
  return (
    <Box
      sx={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        height: '100%',
        padding: 4,
        textAlign: 'center',
      }}
    >
      <Box
        sx={{
          width: 80,
          height: 80,
          borderRadius: '50%',
          bgcolor: '#e3f2fd',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          mb: 3,
        }}
      >
        <PrecisionManufacturingIcon sx={{ fontSize: 40, color: '#1976d2' }} />
      </Box>
      <Typography
        variant="h6"
        sx={{
          fontWeight: 600,
          color: 'text.primary',
          mb: 1,
        }}
      >
        Start a new conversation
      </Typography>
      <Typography
        variant="body2"
        sx={{
          color: 'text.secondary',
          maxWidth: 400,
        }}
      >
        I can help you to create missions, control and monitor drones
      </Typography>
    </Box>
  );
};

// --- Message-type blocks ---

const ReasoningBlock = ({ content }) => (
  <MessageListItem>
    <AccordionBlock
      borderColor={COLORS.neutral}
      detailsSx={{ pt: 0 }}
      header={
        <Typography
          variant="caption"
          sx={{ color: 'text.secondary', fontWeight: 'bold', textTransform: 'uppercase' }}
        >
          Thinking Process
        </Typography>
      }
    >
      <Typography
        variant="body2"
        component="div"
        sx={{
          color: 'text.secondary',
          whiteSpace: 'pre-wrap',
          fontFamily: 'monospace',
          fontSize: '0.8rem',
        }}
      >
        {content}
      </Typography>
    </AccordionBlock>
  </MessageListItem>
);

const FunctionCallBlock = ({ content, name, timestampLabel }) => {
  const isValidateMission = name === 'validate_mission' && content?.mission;
  const hasMissionPlanId = content?.missionPlanId || content?.missionPlanid;
  const isCreateMission = isValidateMission || hasMissionPlanId;

  const handleShowMission = async () => {
    try {
      if (hasMissionPlanId) {
        const endpoint = `/api/missions/plans/show/${content.missionPlanId || content.missionPlanid}`;
        await fetch(endpoint, { method: 'GET' });
      }

      if (isValidateMission) {
        const missionPayload = {
          version: content.mission.version || '3',
          name: content.mission.name || 'Validated Mission',
          route: content.mission.route,
          global_origin: content.mission.global_origin || {
            lat: 41.687222,
            lng: -8.8477450788,
            alt: 0,
          },
        };
        await fetch('/api/missions/showXYZ', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(missionPayload),
        });
      }
    } catch (error) {
      console.error('Error creating mission:', error);
    }
  };

  return (
    <MessageListItem>
      <AccordionBlock
        borderColor={COLORS.info}
        header={
          <AccordionHeader
            label={`Run: ${name}`}
            labelColor={COLORS.infoText}
            timestampLabel={timestampLabel}
            extra={
              isCreateMission && (
                <Button
                  variant="contained"
                  size="small"
                  onClick={(e) => {
                    e.stopPropagation();
                    handleShowMission();
                  }}
                  sx={{
                    bgcolor: COLORS.success,
                    '&:hover': { bgcolor: COLORS.successText },
                    textTransform: 'none',
                    marginLeft: 'auto',
                  }}
                >
                  Show Misión
                </Button>
              )
            }
          />
        }
      >
        <Typography variant="caption" sx={{ color: 'text.secondary', display: 'block', mb: 1 }}>
          Parámetros de entrada:
        </Typography>
        <CodeBlock language="json" value={getPrettyJson(content)} />
      </AccordionBlock>
    </MessageListItem>
  );
};

// Async subagent result appended to the chat. Same accordion vocabulary as a
// tool result — for the reader it IS a result — but labelled by the agent that produced it.
const SubagentResultBlock = ({ content, name, agentName, timestampLabel }) => {
  const subStatus = typeof content === 'object' ? content?.status : null;
  const hasError = subStatus === 'error' || subStatus === 'incomplete';
  const label = `Subagent: ${agentName || name || 'unknown'}${subStatus ? ` — ${subStatus}` : ''}`;

  return (
    <MessageListItem>
      <AccordionBlock
        borderColor={hasError ? COLORS.error : COLORS.purple}
        header={
          <AccordionHeader
            label={label}
            labelColor={hasError ? COLORS.errorText : COLORS.purpleText}
            timestampLabel={timestampLabel}
          />
        }
      >
        {typeof content === 'object' && content?.description && (
          <Typography variant="body2" sx={{ color: 'text.primary', mb: 1 }}>
            {content.description}
          </Typography>
        )}
        <FormattedResult content={content} />
      </AccordionBlock>
    </MessageListItem>
  );
};

const FunctionCallOutputBlock = ({ content, name, timestampLabel }) => {
  const { summary, hasError } = getCachedResultSummary(content);
  const imageData = content?.image_data;
  const imageMime = content?.mime_type || 'image/jpeg';
  const imageSrc = imageData ? `data:${imageMime};base64,${imageData}` : null;

  return (
    <MessageListItem>
      {imageSrc && (
        <Box sx={{ mb: 1, pl: 0.5 }}>
          <img
            src={imageSrc}
            alt={`${name} result`}
            style={{ maxWidth: '100%', maxHeight: 320, borderRadius: 8, display: 'block' }}
          />
        </Box>
      )}
      <AccordionBlock
        borderColor={hasError ? COLORS.error : COLORS.success}
        header={
          <AccordionHeader
            label={summary}
            labelColor={hasError ? COLORS.errorText : COLORS.successText}
            timestampLabel={timestampLabel}
          />
        }
      >
        <Typography variant="caption" sx={{ color: 'text.secondary', display: 'block', mb: 1 }}>
          Resultado:
        </Typography>
        <FormattedResult content={content} />
      </AccordionBlock>
    </MessageListItem>
  );
};

const CodeRenderer = ({ inline, className, children, ...props }) => {
  const match = /language-(\w+)/.exec(className || '');
  if (!inline && match) {
    return <CodeBlock language={match[1]} value={String(children).replace(/\n$/, '')} {...props} />;
  }
  return (
    <code
      className={className}
      style={{
        backgroundColor: 'rgba(0,0,0,0.05)',
        padding: '2px 4px',
        borderRadius: '4px',
        fontFamily: 'monospace',
      }}
      {...props}
    >
      {children}
    </code>
  );
};

const markdownComponents = { code: CodeRenderer };

const MultipartContent = ({ content }) => (
  <Box>
    {content.map((block, i) => {
      if (block.type === 'input_text' && block.text?.trim()) {
        return (
          <Typography
            key={`text-${block.text}-${i}`}
            variant="body1"
            sx={{ whiteSpace: 'pre-wrap', mb: 1 }}
          >
            {block.text}
          </Typography>
        );
      }
      if (block.type === 'input_image') {
        return (
          <Box key={`image-${block.image_url}`} sx={{ mt: 1 }}>
            <img
              src={block.image_url}
              alt="attached"
              style={{ maxWidth: '100%', maxHeight: 300, borderRadius: 8, display: 'block' }}
            />
          </Box>
        );
      }
      return null;
    })}
  </Box>
);

// Standard chat bubble (user or AI text/multipart), with a right-click "fork from here" menu.
const TextMessageBlock = ({
  message,
  chatId,
  role,
  type,
  content,
  status,
  timestampLabel,
  intrinsicSize,
}) => {
  const dispatch = useDispatch();
  const [contextMenu, setContextMenu] = useState(null);

  const isAI = role !== 'user';
  const isError = status === 'error';

  const handleContextMenu = (event) => {
    event.preventDefault();
    setContextMenu({ mouseX: event.clientX, mouseY: event.clientY });
  };

  const handleCloseContextMenu = () => setContextMenu(null);

  const handleFork = async () => {
    handleCloseContextMenu();
    if (!chatId || !message.timestamp) return;
    try {
      await dispatch(forkConversation(chatId, message.timestamp));
    } catch (error) {
      console.error('Error forking conversation:', error);
    }
  };

  return (
    <>
      <ListItem
        onContextMenu={handleContextMenu}
        sx={isAI ? textRowAiSx : textRowUserSx}
        style={{ containIntrinsicSize: intrinsicSize }}
      >
        <Stack
          direction={isAI ? 'row' : 'row-reverse'}
          spacing={1}
          sx={{ alignItems: 'flex-start', width: '100%' }}
        >
          {!isAI && (
            <Avatar sx={{ bgcolor: '#ed6c02', width: 32, height: 32, mt: 0.5 }}>
              <PersonIcon fontSize="small" />
            </Avatar>
          )}
          <Box
            sx={{
              bgcolor: isError ? '#fff3e0' : isAI ? '#ffffff' : '#1976d2',
              color: isError ? '#e65100' : isAI ? 'text.primary' : '#fff',
              borderRadius: '12px',
              pt: 2,
              px: 2,
              pb: 0.5,
              boxShadow: '0 2px 4px rgba(0,0,0,0.08)',
              width: 'fit-content',
              minWidth: '200px',
              border: isError ? '1px solid #ffb74d' : isAI ? '1px solid #f0f0f0' : 'none',
            }}
          >
            {isAI ? (
              <ReactMarkdown remarkPlugins={[remarkGfm]} components={markdownComponents}>
                {content}
              </ReactMarkdown>
            ) : type === 'multipart' ? (
              <MultipartContent content={content} />
            ) : (
              <Typography variant="body1" sx={{ whiteSpace: 'pre-wrap' }}>
                {content}
              </Typography>
            )}
            {timestampLabel && (
              <Typography
                variant="caption"
                sx={{
                  display: 'block',
                  textAlign: 'right',
                  color: isAI ? 'text.disabled' : 'rgba(255,255,255,0.6)',
                  fontSize: '0.65rem',
                  mt: 0,
                }}
              >
                {timestampLabel}
              </Typography>
            )}
          </Box>
        </Stack>
      </ListItem>

      <Menu
        open={contextMenu !== null}
        onClose={handleCloseContextMenu}
        anchorReference="anchorPosition"
        anchorPosition={
          contextMenu !== null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined
        }
      >
        <MuiMenuItem onClick={handleFork} disabled={!chatId}>
          <CallSplitIcon fontSize="small" sx={{ mr: 1 }} />
          Nueva conversación desde aquí
        </MuiMenuItem>
      </Menu>
    </>
  );
};

export const MessageBubble = memo(({ message, chatId }) => {
  const { role, type, content, name, status, agentName, timestampLabel, intrinsicSize } =
    getMessageView(message);

  switch (type) {
    case 'system_directive':
      return null;
    case 'reasoning':
      return <ReasoningBlock content={content} />;
    case 'function_call':
      return <FunctionCallBlock content={content} name={name} timestampLabel={timestampLabel} />;
    case 'subagent_result':
      return (
        <SubagentResultBlock
          content={content}
          name={name}
          agentName={agentName}
          timestampLabel={timestampLabel}
        />
      );
    case 'function_call_output':
      return (
        <FunctionCallOutputBlock content={content} name={name} timestampLabel={timestampLabel} />
      );
    default:
      return (
        <TextMessageBlock
          message={message}
          chatId={chatId}
          role={role}
          type={type}
          content={content}
          status={status}
          timestampLabel={timestampLabel}
          intrinsicSize={intrinsicSize}
        />
      );
  }
});
