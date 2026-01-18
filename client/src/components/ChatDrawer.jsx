import React, { useState, useRef, useEffect, use } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { Rnd } from 'react-rnd';
import { chatActions } from '../store';
import { sendChatMessage } from '../SocketController';
import {
  Drawer,
  IconButton,
  List,
  ListItem,
  ListItemButton,
  ListItemText,
  Toolbar,
  Typography,
  Box,
  Divider,
  TextField,
  Button,
  Avatar,
  Paper,
  AppBar,
  Stack,
  Select,
  MenuItem,
  FormControl,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import DeleteIcon from '@mui/icons-material/Delete';
import AddIcon from '@mui/icons-material/Add';
import ChevronLeftIcon from '@mui/icons-material/ChevronLeft';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import SendIcon from '@mui/icons-material/Send';
import PersonIcon from '@mui/icons-material/Person';
import SmartToyIcon from '@mui/icons-material/SmartToy';
import ClearIcon from '@mui/icons-material/Clear';
import ReplayIcon from '@mui/icons-material/Replay';
import MicIcon from '@mui/icons-material/Mic';
import StopIcon from '@mui/icons-material/Stop';
import CircularProgressIcon from '@mui/material/CircularProgress';
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { vscDarkPlus } from 'react-syntax-highlighter/dist/esm/styles/prism';
import ContentCopyIcon from '@mui/icons-material/ContentCopy';
import CheckIcon from '@mui/icons-material/Check';
import Tooltip from '@mui/material/Tooltip';

const useStyles = makeStyles()((theme) => ({
  panel: {
    background: '#f0f0f0',
    borderLeft: '2px solid #ccc',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    flexDirection: 'column',
    boxSizing: 'border-box', // Asegura que el padding y borde no afecten el ancho total
    zIndex: 9,
    cursor: 'default !important', // Anula el cursor "move" de react-rnd
    '& *': {
      cursor: 'default', // Asegura que todos los elementos internos tengan cursor normal
    },
  },
  container: {
    width: '100%',
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
    overflow: 'hidden',
    borderRadius: 2,
    border: '1px solid #e0e0e0',
  },
  drawer: {
    width: theme.dimensions.chatDrawerWidth,
  },
  toolbar: {
    paddingLeft: theme.spacing(2),
    paddingRight: theme.spacing(2),
  },
  title: {
    flexGrow: 1,
  },
  ChatContainer: {
    display: 'flex',
    flexDirection: 'column',
    height: '100%',
    padding: theme.spacing(2),
    boxSizing: 'border-box',
  },
  MessagesArea: {
    flexGrow: 1,
    overflowY: 'auto',
    display: 'flex',
    flexDirection: 'column',
    marginBottom: theme.spacing(2),
    paddingRight: theme.spacing(1),
  },
  MessageBubble: {},
  MessageOptions: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1),
    margin: theme.spacing(2),
  },
  MessageInputArea: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
  },
}));

const initialOptions = [
  'crea una mission rapida para la inspecion de los aerogeneadores que se encuentran offshore en viena de castelo',
  'crea una mission para la inspecion de los aerogeneadores de la linea A en viena de castelo',
  'has una mission detallada para el aerogeneador 5 de la linea A en viena de castelo',
  '¿Qué drones están disponibles?',
  'Crea una misión para que un dron vuele 100m al norte y regrese.',
  'Selecciona un dron y haz que se mantenga en vuelo estacionario a 10 metros de altura.',
];

const convertMsg = (msg) => {
  if (!msg || !msg.message) return { role: 'assistant', type: 'error', content: 'Invalid message' };

  if (typeof msg.message.content === 'string' && msg.message.type === 'text') {
    // Explicit text type check if present, or default
    return {
      role: msg.message.role,
      content: msg.message.content,
      type: 'text',
    };
  }
  // Handle legacy/simple string messages (assuming they are text)
  if (typeof msg.message.content === 'string' && !msg.message.type) {
    return {
      role: msg.message.role,
      content: msg.message.content,
      type: 'text',
    };
  }

  if (msg.message.type === 'reasoning') {
    return { role: 'assistant', type: 'reasoning', content: msg.message.content };
  }

  if (msg.message.type === 'message') {
    // Sometimes messages come in an array
    let contentText = '';
    if (typeof msg.message.content === 'string') {
      contentText = msg.message.content;
    } else if (Array.isArray(msg.message.content)) {
      // Attempt to find text part
      const textPart = msg.message.content.find((c) => c.type === 'text');
      contentText = textPart ? textPart.text : msg.message.content[0]?.text || '';
    }
    return { role: msg.message.role, type: 'text', content: contentText };
  }

  if (msg.message.type === 'function_call' || msg.message.type === 'tool_call') {
    // Parse arguments if they're a string
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
    // Parse output if it's a string
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
  };
};

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

const WelcomeMessage = () => {
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

const MessageBubble = ({ message }) => {
  const { role, type, content, name } = convertMsg(message);
  const isAI = role !== 'user';

  // Common styles for compact accordions
  const accordionStyle = (borderColor, iconColor) => ({
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

  // Specific rendering for system/tool/reasoning blocks (Collapsible)
  if (type === 'reasoning') {
    return (
      <ListItem sx={{ mb: 1, display: 'block', px: 2 }}>
        <Accordion sx={accordionStyle('#9e9e9e', '#757575')}>
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

  if (type === 'function_call') {
    return (
      <ListItem sx={{ mb: 1, display: 'block', px: 2 }}>
        <Accordion sx={accordionStyle('#2196f3', '#1976d2')} defaultExpanded={false}>
          <AccordionSummary expandIcon={<ExpandMoreIcon fontSize="small" />} sx={summaryStyle}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, width: '100%' }}>
              <Typography
                variant="caption"
                sx={{ color: '#1976d2', fontWeight: 'bold', fontFamily: 'monospace', fontSize: '0.85rem' }}
              >
                🛠️ Ejecutando: {name}
              </Typography>
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

  if (type === 'function_call_output') {
    // Try to detect if the result contains mission data or error
    let resultSummary = '✅ Completado';
    let hasError = false;

    if (typeof content === 'object') {
      if (content.error) {
        resultSummary = '❌ Error';
        hasError = true;
      } else if (content.success === false) {
        resultSummary = '⚠️ Falló';
        hasError = true;
      } else if (content.mission || content.missionId) {
        resultSummary = '✅ Misión creada';
      } else if (content.devices || Array.isArray(content)) {
        resultSummary = '✅ Datos obtenidos';
      }
    }

    return (
      <ListItem sx={{ mb: 1, display: 'block', px: 2 }}>
        <Accordion sx={accordionStyle(hasError ? '#f44336' : '#4caf50', hasError ? '#d32f2f' : '#388e3c')}>
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
            <CodeBlock
              language="json"
              value={typeof content === 'object' ? JSON.stringify(content, null, 2) : content}
            />
          </AccordionDetails>
        </Accordion>
      </ListItem>
    );
  }

  // Standard Text Message (User or AI)
  return (
    <ListItem
      sx={{
        justifyContent: isAI ? 'flex-start' : 'flex-end',
        mb: 2,
        alignItems: 'flex-start',
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
            bgcolor: isAI ? '#ffffff' : '#1976d2',
            color: isAI ? 'text.primary' : '#fff',
            borderRadius: '12px',
            p: 2,
            boxShadow: '0 2px 4px rgba(0,0,0,0.08)',
            width: 'fit-content',
            minWidth: '200px',
            border: isAI ? '1px solid #f0f0f0' : 'none',
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
  );
};

const ChatDrawer = ({ open, onClose }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  // Get chat state from Redux
  const activeChatId = useSelector((state) => state.chat.activeChatId);
  const activeConversation = useSelector((state) => (activeChatId ? state.chat.conversations[activeChatId] : null));
  const messages = activeConversation?.messages || [];
  const loading = useSelector((state) => state.chat.loading);
  const availableChats = useSelector((state) => state.chat.availableChats);

  // Local UI state
  const [inputValue, setInputValue] = useState('');
  const [showOptions, setShowOptions] = useState(true);
  const [isRecording, setIsRecording] = useState(false);
  const [mediaRecorder, setMediaRecorder] = useState(null);
  const audioChunksRef = useRef([]);
  const audioPlayerRef = useRef(null);

  const messagesEndRef = useRef(null);
  const [panelState, setPanelState] = useState({
    width: 500, // Ancho inicial
    height: window.innerHeight, // Altura completa
    x: window.innerWidth - 500,
    y: 0,
  });

  // Fetch available chats from server
  const fetchAvailableChats = async () => {
    try {
      dispatch(chatActions.setLoading({ key: 'loadingChatList', value: true }));
      const response = await fetch('/api/chat/chats');
      if (response.ok) {
        const data = await response.json();
        dispatch(chatActions.setAvailableChats(data.chats || []));
      }
    } catch (error) {
      console.error('Error fetching chats:', error);
    } finally {
      dispatch(chatActions.setLoading({ key: 'loadingChatList', value: false }));
    }
  };

  // Load chat history from server
  const loadChatHistory = async (chatId) => {
    try {
      dispatch(chatActions.setLoading({ key: 'loadingHistory', value: true }));
      const response = await fetch(`/api/chat/history/${chatId}`);
      if (response.ok) {
        const data = await response.json();
        if (data.messages && data.messages.length > 0) {
          dispatch(chatActions.setMessages({ chatId, messages: data.messages }));
          setShowOptions(false);
        }
      }
    } catch (error) {
      console.error('Error loading chat history:', error);
    } finally {
      dispatch(chatActions.setLoading({ key: 'loadingHistory', value: false }));
    }
  };

  // Handle chat selection change
  const handleChatChange = async (event) => {
    const newChatId = event.target.value;
    if (newChatId && newChatId !== activeChatId) {
      dispatch(chatActions.setActiveChat(newChatId));
      // Load history if not already loaded
      const existingConversation = availableChats.find((c) => c.id === newChatId);
      if (existingConversation) {
        await loadChatHistory(newChatId);
      }
    }
  };

  // Fetch chats on component mount
  useEffect(() => {
    if (open) {
      fetchAvailableChats();
    }
  }, [open]);

  // Initialize new chat on mount with unique ID
  useEffect(() => {
    if (!activeChatId) {
      // Create a unique chat ID based on timestamp
      const newChatId = `chat_${Date.now()}`;
      dispatch(chatActions.setActiveChat(newChatId));
    }
  }, [activeChatId, dispatch]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (messageToSend, fromAudio = false) => {
    console.log('Enviando mensaje:', messageToSend);
    if (!messageToSend.trim() || loading.sendingMessage) return;

    try {
      // Set loading state
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: true }));

      // Add user message to Redux immediately
      const userMessage = {
        role: 'user',
        content: messageToSend,
        timestamp: new Date().toISOString(),
      };

      dispatch(
        chatActions.addMessage({
          chatId: activeChatId,
          message: userMessage,
        })
      );

      // Clear input
      setInputValue('');
      setShowOptions(false);

      // Send via WebSocket (NOT HTTP!)
      sendChatMessage(activeChatId, messageToSend);

      // Store if we should play audio response
      if (fromAudio) {
        window.shouldPlayAudioResponse = true;
      }

      // Note: Response will come via WebSocket and be handled by SocketController
    } catch (error) {
      console.error('Error sending message:', error);
      dispatch(chatActions.setError(error.message));

      // Show error message in chat
      dispatch(
        chatActions.addMessage({
          chatId: activeChatId,
          message: {
            role: 'assistant',
            content: 'Error: No se pudo enviar el mensaje. Por favor verifica la conexión WebSocket.',
            timestamp: new Date().toISOString(),
          },
        })
      );
    } finally {
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: false }));
    }
  };

  const handleKeyPress = (event) => {
    if (event.key === 'Enter' && !event.shiftKey) {
      event.preventDefault();
      handleSendMessage(inputValue);
    }
  };
  const handleDrawerClose = () => {
    onClose(false);
  };
  const handleOptionClick = (optionText) => {
    console.log(` ${optionText}`);
    handleSendMessage(optionText); // Enviar la opción seleccionada como un mensaje del usuario
  };

  const clearChat = () => {
    // Create a new chat with unique ID
    const newChatId = `chat_${Date.now()}`;
    dispatch(
      chatActions.createChat({
        chatId: newChatId,
        name: `Chat ${new Date().toLocaleString()}`,
      })
    );

    setShowOptions(true);
    // Refresh available chats list after a short delay
    setTimeout(() => fetchAvailableChats(), 500);
  };

  const startRecording = async () => {
    try {
      // Verificar si el navegador soporta la API de getUserMedia
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        alert('Tu navegador no soporta grabación de audio. Por favor, usa un navegador moderno.');
        return;
      }

      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });

      // Verificar que el stream tiene tracks de audio activos
      const audioTracks = stream.getAudioTracks();
      if (audioTracks.length === 0) {
        alert('No se detectó ningún micrófono. Por favor, conecta un micrófono e intenta de nuevo.');
        stream.getTracks().forEach((track) => track.stop());
        return;
      }

      // Verificar que el micrófono está habilitado
      const audioTrack = audioTracks[0];
      if (!audioTrack.enabled) {
        alert('El micrófono está deshabilitado. Por favor, habilítalo en la configuración del sistema.');
        stream.getTracks().forEach((track) => track.stop());
        return;
      }

      // Verificar el nivel de audio para detectar si el micrófono está funcionando
      const AudioContextClass = window.AudioContext || window['webkitAudioContext'];
      const audioContext = new AudioContextClass();
      const analyser = audioContext.createAnalyser();
      const microphone = audioContext.createMediaStreamSource(stream);
      microphone.connect(analyser);
      analyser.fftSize = 256;
      const dataArray = new Uint8Array(analyser.frequencyBinCount);

      // Verificar que hay señal de audio después de 500ms
      setTimeout(() => {
        analyser.getByteFrequencyData(dataArray);
        const average = dataArray.reduce((a, b) => a + b) / dataArray.length;

        if (average < 1) {
          console.warn('⚠️ No se detecta señal del micrófono. Puede estar desactivado o silenciado.');
        }
      }, 500);

      const recorder = new MediaRecorder(stream);
      audioChunksRef.current = [];

      recorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          audioChunksRef.current.push(event.data);
        }
      };

      recorder.onstop = async () => {
        const audioBlob = new Blob(audioChunksRef.current, { type: 'audio/webm' });

        // Verificar que se grabó algo
        if (audioBlob.size < 100) {
          alert('No se grabó ningún audio. Verifica que el micrófono esté habilitado y no silenciado.');
          stream.getTracks().forEach((track) => track.stop());
          audioContext.close();
          return;
        }

        await sendAudioMessage(audioBlob);

        // Detener todos los tracks del stream para liberar el micrófono
        stream.getTracks().forEach((track) => track.stop());
        audioContext.close();
      };

      recorder.start();
      setMediaRecorder(recorder);
      setIsRecording(true);
    } catch (error) {
      console.error('Error al acceder al micrófono:', error);

      // Mensajes de error más específicos según el tipo de error
      if (error.name === 'NotAllowedError' || error.name === 'PermissionDeniedError') {
        alert('Permiso denegado. Por favor, permite el acceso al micrófono en la configuración del navegador.');
      } else if (error.name === 'NotFoundError' || error.name === 'DevicesNotFoundError') {
        alert('No se encontró ningún micrófono. Por favor, conecta un micrófono e intenta de nuevo.');
      } else if (error.name === 'NotReadableError' || error.name === 'TrackStartError') {
        alert(
          'El micrófono está siendo usado por otra aplicación o está deshabilitado en el sistema. Por favor, cierra otras aplicaciones que puedan estar usando el micrófono.'
        );
      } else {
        alert(
          'No se pudo acceder al micrófono: ' +
            error.message +
            '\n\nVerifica que:\n- El micrófono esté conectado\n- El micrófono esté habilitado en la configuración del sistema\n- No esté siendo usado por otra aplicación'
        );
      }
    }
  };

  const stopRecording = () => {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
      mediaRecorder.stop();
      setIsRecording(false);
      setMediaRecorder(null);
    }
  };

  const sendAudioMessage = async (audioBlob) => {
    setIsLoading(true);

    try {
      // Verificar el tamaño del audio antes de enviarlo
      if (audioBlob.size < 1000) {
        alert('El audio es demasiado corto. Por favor, graba un mensaje más largo.');
        setIsLoading(false);
        return;
      }

      // Crear FormData para enviar el audio
      const formData = new FormData();
      formData.append('audio', audioBlob, 'audio.webm');

      // Enviar el audio al servidor para transcripción
      const transcriptionResponse = await fetch('/api/chat/stt', {
        method: 'POST',
        body: formData,
      });

      const responseData = await transcriptionResponse.json();

      if (!transcriptionResponse.ok) {
        // Si es una alucinación detectada, mostrar mensaje específico
        if (responseData.hallucination) {
          alert(responseData.error || 'No se detectó voz clara en el audio. Por favor, habla más cerca del micrófono.');
        } else {
          throw new Error(responseData.error || 'Error al transcribir el audio');
        }
        setIsLoading(false);
        return;
      }

      const { text } = responseData;

      if (text && text.trim()) {
        // Enviar el texto transcrito como mensaje, indicando que viene de audio
        handleSendMessage(text, true);
      } else {
        alert('No se pudo transcribir el audio. Por favor, intenta de nuevo.');
        setIsLoading(false);
      }
    } catch (error) {
      console.error('Error al procesar el audio:', error);
      alert('Hubo un error al procesar el audio. Asegúrate de hablar claramente y cerca del micrófono.');
      setIsLoading(false);
    }
  };

  const textToSpeech = async (text) => {
    try {
      const response = await fetch('/api/chat/tts', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ text }),
      });

      if (!response.ok) {
        throw new Error('Error al generar audio');
      }

      const audioBlob = await response.blob();
      const audioUrl = URL.createObjectURL(audioBlob);

      // Detener cualquier audio que esté reproduciéndose
      if (audioPlayerRef.current) {
        audioPlayerRef.current.pause();
        audioPlayerRef.current = null;
      }

      // Crear y reproducir el nuevo audio
      const audio = new Audio(audioUrl);
      audioPlayerRef.current = audio;

      audio.play().catch((error) => {
        console.error('Error al reproducir audio:', error);
      });

      // Limpiar el URL cuando termine de reproducir
      audio.onended = () => {
        URL.revokeObjectURL(audioUrl);
        audioPlayerRef.current = null;
      };
    } catch (error) {
      console.error('Error en text-to-speech:', error);
      // No mostrar alert para no interrumpir el flujo del chat
    }
  };

  useEffect(() => {
    const handleWindowResize = () => {
      setPanelState((prevState) => ({
        ...prevState,
        x: window.innerWidth - prevState.width,
      }));
    };

    window.addEventListener('resize', handleWindowResize);
    return () => {
      window.removeEventListener('resize', handleWindowResize);
    };
  }, []);

  const handleResizeStop = (e, direction, ref, delta, position) => {
    // Actualizamos el estado con el nuevo ancho y la nueva posición.
    setPanelState({
      width: parseInt(ref.style.width, 10),
      height: '100vh',
      ...position, // 'position' contiene los nuevos valores de {x, y}
    });
  };

  return (
    <>
      {open && (
        <Rnd
          className={classes.panel}
          size={{ width: panelState.width, height: panelState.height }}
          position={{ x: panelState.x, y: panelState.y }}
          minWidth={300}
          maxWidth={window.innerWidth / 2}
          enableResizing={{ left: true, right: false }}
          dragAxis="none"
          onResizeStop={handleResizeStop}
          disableDragging={true}
          style={{ cursor: 'default' }}
        >
          <Paper className={classes.container}>
            <AppBar position="static" sx={{ backgroundColor: '#673ab7' }}>
              <Toolbar className={classes.toolbar} disableGutters>
                <Typography variant="h6" className={classes.title}>
                  {'Chat Assistant'}
                </Typography>
                <IconButton size="small" color="inherit" onClick={clearChat} sx={{ mr: 0.5 }}>
                  <ReplayIcon fontSize="small" />
                </IconButton>
                <IconButton size="small" color="inherit" onClick={handleDrawerClose}>
                  <ChevronRightIcon fontSize="small" />
                </IconButton>
              </Toolbar>
            </AppBar>

            {/* Chat Selector */}
            <Box sx={{ p: 1, borderBottom: '1px solid #e0e0e0', backgroundColor: '#fafafa' }}>
              <Stack direction="row" spacing={1} alignItems="center">
                <FormControl size="small" sx={{ flexGrow: 1 }}>
                  <Select
                    value={activeChatId || ''}
                    onChange={handleChatChange}
                    displayEmpty
                    disabled={loading.loadingChatList}
                    sx={{
                      bgcolor: '#ffffff',
                      '& .MuiSelect-select': {
                        py: 1,
                      },
                    }}
                  >
                    {activeChatId && !availableChats.find((c) => c.id === activeChatId) && (
                      <MenuItem value={activeChatId}>
                        {activeConversation?.name || 'New Chat'}
                      </MenuItem>
                    )}
                    {availableChats.map((chat) => (
                      <MenuItem key={chat.id} value={chat.id}>
                        {chat.name || `Chat ${new Date(chat.createdAt).toLocaleDateString()}`}
                      </MenuItem>
                    ))}
                    {availableChats.length === 0 && !activeChatId && (
                      <MenuItem value="" disabled>
                        No chats available
                      </MenuItem>
                    )}
                  </Select>
                </FormControl>
                <Tooltip title="New Chat">
                  <IconButton
                    size="small"
                    onClick={clearChat}
                    sx={{
                      bgcolor: '#673ab7',
                      color: 'white',
                      '&:hover': { bgcolor: '#5e35b1' },
                    }}
                  >
                    <AddIcon fontSize="small" />
                  </IconButton>
                </Tooltip>
              </Stack>
            </Box>

            {/* Messages Area */}
            <Box className={classes.MessagesArea}>
              {messages.length === 0 ? (
                <WelcomeMessage />
              ) : (
                <>
                  {messages.map((msg, index) => (
                    <MessageBubble key={index} message={msg} />
                  ))}
                  {loading.sendingMessage && (
                    <ListItem sx={{ justifyContent: 'flex-start', py: 1 }}>
                      <Avatar sx={{ bgcolor: '#1976d2', width: 32, height: 32, mr: 1 }}>
                        <SmartToyIcon fontSize="small" />
                      </Avatar>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                        <CircularProgressIcon size={16} />
                        <Typography variant="body2" color="text.secondary">
                          Thinking...
                        </Typography>
                      </Box>
                    </ListItem>
                  )}
                </>
              )}
              <div ref={messagesEndRef} />
            </Box>

            <Box sx={{ p: 2, borderTop: '1px solid #e0e0e0', backgroundColor: '#ffffff' }}>
              {/* Optios messages */}
              {showOptions && messages.length === 0 && (
                <Box sx={{ mb: 2 }}>
                  <Stack direction="row" flexWrap="wrap" spacing={1} useFlexGap>
                    {initialOptions.map((option, index) => (
                      <Button
                        key={index}
                        variant="outlined"
                        onClick={() => handleOptionClick(option)}
                        sx={{
                          justifyContent: 'flex-start',
                          textTransform: 'none',
                          borderRadius: '20px',
                          borderColor: '#bdbdbd',
                          fontWeight: 600,
                          color: '#424242',
                          '&:hover': {
                            borderColor: '#673ab7',
                            backgroundColor: '#f3e5f5',
                          },
                        }}
                      >
                        {option}
                      </Button>
                    ))}
                  </Stack>
                </Box>
              )}

              {/* Input Area */}
              <Stack direction="row" spacing={1}>
                <TextField
                  fullWidth
                  multiline
                  maxRows={3}
                  placeholder="Escribe tu mensaje..."
                  value={inputValue}
                  onChange={(e) => setInputValue(e.target.value)}
                  onKeyUp={handleKeyPress}
                  disabled={loading.sendingMessage || isRecording}
                  variant="outlined"
                  size="small"
                  sx={{
                    '& .MuiOutlinedInput-root': {
                      borderRadius: 2,
                      bgcolor: '#f8f9fa',
                    },
                    '& .MuiInputBase-input': {
                      height: 'auto',
                    },
                  }}
                />
                <IconButton
                  onClick={isRecording ? stopRecording : startRecording}
                  disabled={loading.sendingMessage}
                  sx={{
                    bgcolor: isRecording ? '#f44336' : '#4caf50',
                    color: 'white',
                    '&:hover': { bgcolor: isRecording ? '#d32f2f' : '#388e3c' },
                    '&:disabled': { bgcolor: '#e0e0e0' },
                    width: 40,
                    height: 40,
                    animation: isRecording ? 'pulse 1.5s ease-in-out infinite' : 'none',
                    '@keyframes pulse': {
                      '0%': { transform: 'scale(1)', opacity: 1 },
                      '50%': { transform: 'scale(1.1)', opacity: 0.8 },
                      '100%': { transform: 'scale(1)', opacity: 1 },
                    },
                  }}
                >
                  {isRecording ? <StopIcon fontSize="small" /> : <MicIcon fontSize="small" />}
                </IconButton>
                <IconButton
                  onClick={() => handleSendMessage(inputValue)}
                  disabled={!inputValue.trim() || loading.sendingMessage || isRecording}
                  sx={{
                    bgcolor: '#1976d2',
                    color: 'white',
                    '&:hover': { bgcolor: '#1565c0' },
                    '&:disabled': { bgcolor: '#e0e0e0' },
                    width: 40,
                    height: 40,
                  }}
                >
                  {loading.sendingMessage ? (
                    <CircularProgressIcon size={20} color="inherit" />
                  ) : (
                    <SendIcon fontSize="small" />
                  )}
                </IconButton>
              </Stack>
            </Box>
          </Paper>
        </Rnd>
      )}
    </>
  );
};

export default ChatDrawer;
