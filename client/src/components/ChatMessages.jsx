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
import { forkConversation } from '../store/chat';

// --- Data transformation utils ---

const convertMsg = (msg) => {
  if (!msg || !msg.message) return { role: 'assistant', type: 'error', content: 'Invalid message', status: 'error' };

  const status = msg.message.status || null;

  if (typeof msg.message.content === 'string' && (msg.message.type === 'text' || !msg.message.type)) {
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
      } catch (e) {
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

  if (msg.message.type === 'function_call_output') {
    let output = msg.message.output || msg.message.content;
    if (typeof output === 'string') {
      try {
        output = JSON.parse(output);
      } catch (e) {
        // Keep as string if not valid JSON
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
    content: typeof msg.message.content === 'string' ? msg.message.content : JSON.stringify(msg.message.content),
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
    const textParts = content.content
      .filter((item) => item.type === 'text' && item.text)
      .map((item) => {
        let text = item.text;
        if (typeof text === 'string') {
          text = text.replace(/\\n/g, '\n').replace(/\\t/g, '\t').replace(/\\"/g, '"').replace(/\\\\/g, '\\');
        }
        return text;
      });

    if (textParts.length > 0) {
      return textParts.join('\n');
    }
  }

  if (content.text && typeof content.text === 'string') {
    return content.text.replace(/\\n/g, '\n').replace(/\\t/g, '\t').replace(/\\"/g, '"').replace(/\\\\/g, '\\');
  }

  return JSON.stringify(content, null, 2);
};

const detectContentLanguage = (content) => {
  if (typeof content === 'string') {
    const trimmed = content.trim();
    if ((trimmed.startsWith('{') && trimmed.endsWith('}')) || (trimmed.startsWith('[') && trimmed.endsWith(']'))) {
      return 'json';
    }
    return 'text';
  }
  return 'json';
};

// --- Presentation components ---

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

export const MessageBubble = memo(({ message, chatId }) => {
  const dispatch = useDispatch();
  const { role, type, content, name, status } = convertMsg(message);

  const [contextMenu, setContextMenu] = useState(null);

  const handleContextMenu = (event) => {
    event.preventDefault();
    setContextMenu({ mouseX: event.clientX, mouseY: event.clientY });
  };

  const handleCloseContextMenu = () => {
    setContextMenu(null);
  };

  const handleFork = async () => {
    handleCloseContextMenu();
    if (!chatId || !message.timestamp) return;
    try {
      await dispatch(forkConversation(chatId, message.timestamp));
    } catch (error) {
      console.error('Error forking conversation:', error);
    }
  };
  const isAI = role !== 'user';
  const isError = status === 'error';

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

  // Reasoning block
  if (type === 'reasoning') {
    return (
      <ListItem sx={{ mb: 1, display: 'block', px: 2 }}>
        <Accordion sx={accordionStyle('#9e9e9e')}>
          <AccordionSummary expandIcon={<ExpandMoreIcon fontSize="small" />} sx={summaryStyle}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
              <Typography
                variant="caption"
                sx={{ color: 'text.secondary', fontWeight: 'bold', textTransform: 'uppercase' }}
              >
                Thinking Process
              </Typography>
            </Box>
          </AccordionSummary>
          <AccordionDetails sx={{ pt: 0, pb: 1, pl: 2, pr: 1 }}>
            <Typography
              variant="body2"
              component="div"
              sx={{ color: 'text.secondary', whiteSpace: 'pre-wrap', fontFamily: 'monospace', fontSize: '0.8rem' }}
            >
              {content}
            </Typography>
          </AccordionDetails>
        </Accordion>
      </ListItem>
    );
  }

  // Function call block
  if (type === 'function_call') {
    const hasMissionData = content?.missionData;
    const hasMissionDataXYZ = content?.missionDataXYZ;
    const isValidateMission = name === 'validate_mission_collisions' && content?.mission;
    const isCreateMission = hasMissionData || hasMissionDataXYZ || isValidateMission;

    const handleCreateMission = async () => {
      try {
        let endpoint;
        let missionPayload;

        if (hasMissionDataXYZ) {
          endpoint = '/api/missions/showXYZ';
          missionPayload = content.missionDataXYZ;
        } else if (hasMissionData) {
          endpoint = '/api/missions/';
          missionPayload = content.missionData;
        } else if (isValidateMission) {
          endpoint = '/api/missions/showXYZ';
          missionPayload = {
            version: content.mission.version || '3',
            name: content.mission.name || 'Validated Mission',
            route: content.mission.route,
            origin_global: content.origin_global || { lat: 41.68722260607747, lng: -8.847745078804891, alt: 0 },
            chat_id: content.chat_id,
          };
        } else {
          return;
        }

        const response = await fetch(endpoint, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(missionPayload),
        });

        if (response.ok) {
          const result = await response.json();
          console.log(`Misión creada exitosamente! ID: ${result.id || 'N/A'}`);
        } else {
          const error = await response.json();
          console.log(`Error al crear misión: ${error.message || response.statusText}`);
        }
      } catch (error) {
        console.error('Error creating mission:', error);
        console.log(`Error al crear misión: ${error.message}`);
      }
    };

    return (
      <ListItem sx={{ mb: 1, display: 'block', px: 2 }}>
        <Accordion sx={accordionStyle('#2196f3')} defaultExpanded={false}>
          <AccordionSummary expandIcon={<ExpandMoreIcon fontSize="small" />} sx={summaryStyle}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, width: '100%' }}>
              <Typography
                variant="caption"
                sx={{ color: '#1976d2', fontWeight: 'bold', fontFamily: 'monospace', fontSize: '0.85rem' }}
              >
                Ejecutando: {name}
              </Typography>
              {isCreateMission && (
                <Button
                  variant="contained"
                  size="small"
                  onClick={(e) => {
                    e.stopPropagation();
                    handleCreateMission();
                  }}
                  sx={{
                    bgcolor: '#4caf50',
                    '&:hover': { bgcolor: '#388e3c' },
                    textTransform: 'none',
                    marginLeft: 'auto',
                  }}
                >
                  Show Misión
                </Button>
              )}
            </Box>
          </AccordionSummary>
          <AccordionDetails sx={{ pt: 1, pb: 1, pl: 2, pr: 1 }}>
            <Typography variant="caption" sx={{ color: 'text.secondary', display: 'block', mb: 1 }}>
              Parámetros de entrada:
            </Typography>
            <CodeBlock
              language="json"
              value={typeof content === 'object' ? JSON.stringify(content, null, 2) : content}
            />
          </AccordionDetails>
        </Accordion>
      </ListItem>
    );
  }

  // Function call output block
  if (type === 'function_call_output') {
    let resultSummary = 'Completado';
    let hasError = false;

    if (typeof content === 'object') {
      if (content.error) {
        resultSummary = 'Error';
        hasError = true;
      } else if (content.success === false) {
        resultSummary = 'Falló';
        hasError = true;
      } else if (content.mission || content.missionId) {
        resultSummary = 'Misión creada';
      } else if (content.devices || Array.isArray(content)) {
        resultSummary = 'Datos obtenidos';
      }
    }

    return (
      <ListItem sx={{ mb: 1, display: 'block', px: 2 }}>
        <Accordion sx={accordionStyle(hasError ? '#f44336' : '#4caf50')}>
          <AccordionSummary expandIcon={<ExpandMoreIcon fontSize="small" />} sx={summaryStyle}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, width: '100%' }}>
              <Typography
                variant="caption"
                sx={{
                  color: hasError ? '#d32f2f' : '#388e3c',
                  fontWeight: 'bold',
                  fontFamily: 'monospace',
                  fontSize: '0.85rem',
                }}
              >
                {resultSummary}
              </Typography>
            </Box>
          </AccordionSummary>
          <AccordionDetails sx={{ pt: 1, pb: 1, pl: 2, pr: 1 }}>
            <Typography variant="caption" sx={{ color: 'text.secondary', display: 'block', mb: 1 }}>
              Resultado:
            </Typography>
            {(() => {
              const formattedContent = formatMcpContent(content);
              const language = detectContentLanguage(formattedContent);
              return <CodeBlock language={language} value={formattedContent} />;
            })()}
          </AccordionDetails>
        </Accordion>
      </ListItem>
    );
  }

  // Standard text message (User or AI)
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
        <Stack direction={isAI ? 'row' : 'row-reverse'} alignItems="flex-start" spacing={1} sx={{ width: '100%' }}>
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
              p: 2,
              boxShadow: '0 2px 4px rgba(0,0,0,0.08)',
              width: 'fit-content',
              minWidth: '200px',
              border: isError ? '1px solid #ffb74d' : isAI ? '1px solid #f0f0f0' : 'none',
            }}
          >
            {isAI ? (
              <ReactMarkdown
                remarkPlugins={[remarkGfm]}
                components={{
                  code({ node, inline, className, children, ...props }) {
                    const match = /language-(\w+)/.exec(className || '');
                    return !inline && match ? (
                      <CodeBlock language={match[1]} value={String(children).replace(/\n$/, '')} {...props} />
                    ) : (
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
                  },
                }}
              >
                {content}
              </ReactMarkdown>
            ) : (
              <Typography variant="body1" sx={{ whiteSpace: 'pre-wrap' }}>
                {content}
              </Typography>
            )}
          </Box>
        </Stack>
      </ListItem>

      <Menu
        open={contextMenu !== null}
        onClose={handleCloseContextMenu}
        anchorReference="anchorPosition"
        anchorPosition={contextMenu !== null ? { top: contextMenu.mouseY, left: contextMenu.mouseX } : undefined}
      >
        <MuiMenuItem onClick={handleFork} disabled={!chatId}>
          <CallSplitIcon fontSize="small" sx={{ mr: 1 }} />
          Nueva conversación desde aquí
        </MuiMenuItem>
      </Menu>
    </>
  );
});
