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
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { vscDarkPlus } from 'react-syntax-highlighter/dist/esm/styles/prism';
import { forkConversation } from '../../store/chat';

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

const CodeBlock = ({ language, value }) => {
  const [copied, setCopied] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(value);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <Box sx={{ position: 'relative', borderRadius: 1, overflow: 'hidden', my: 1 }}>
      <Box
        sx={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          bgcolor: '#1e1e1e',
          color: '#e0e0e0',
          px: 2,
          py: 0.5,
          fontSize: '0.75rem',
          borderBottom: '1px solid #333',
        }}
      >
        <Typography variant="caption" sx={{ fontFamily: 'monospace' }}>
          {language || 'text'}
        </Typography>
        <Tooltip title={copied ? 'Copiado!' : 'Copiar'}>
          <IconButton onClick={handleCopy} size="small" sx={{ color: '#e0e0e0', p: 0.5 }}>
            {copied ? <CheckIcon fontSize="inherit" /> : <ContentCopyIcon fontSize="inherit" />}
          </IconButton>
        </Tooltip>
      </Box>
      <SyntaxHighlighter
        language={language || 'text'}
        style={vscDarkPlus}
        customStyle={{ margin: 0, borderRadius: '0 0 4px 4px', fontSize: '0.85rem' }}
      >
        {value}
      </SyntaxHighlighter>
    </Box>
  );
};

const FormattedResult = ({ content }) => {
  const formatted = formatMcpContent(content);
  return <CodeBlock language={detectContentLanguage(formatted)} value={formatted} />;
};

// A message row that hosts a single accordion (reasoning / tool call / tool result).
const MessageListItem = ({ children }) => (
  <ListItem sx={{ mb: 1, display: 'block', px: 2 }}>{children}</ListItem>
);

const AccordionBlock = ({ borderColor, header, detailsSx, children }) => (
  <Accordion sx={accordionStyle(borderColor)}>
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
        <CodeBlock
          language="json"
          value={typeof content === 'object' ? JSON.stringify(content, null, 2) : content}
        />
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
  const { summary, hasError } = getResultSummary(content);
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
const TextMessageBlock = ({ message, chatId, role, type, content, status }) => {
  const dispatch = useDispatch();
  const [contextMenu, setContextMenu] = useState(null);

  const isAI = role !== 'user';
  const isError = status === 'error';
  const timestampLabel = formatTimestamp(message.timestamp);

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
        sx={{
          justifyContent: isAI ? 'flex-start' : 'flex-end',
          mb: 2,
          alignItems: 'flex-start',
          cursor: 'context-menu',
        }}
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
  const { role, type, content, name, status, agentName } = convertMsg(message);
  const timestampLabel = formatTimestamp(message.timestamp);

  switch (type) {
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
        />
      );
  }
});
