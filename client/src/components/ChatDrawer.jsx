import React, { useState, useRef, useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { Rnd } from 'react-rnd';
import { chatActions } from '../store';
import { sendChatMessage } from '../SocketController';
import {
  IconButton,
  ListItem,
  Toolbar,
  Typography,
  Box,
  Avatar,
  Paper,
  AppBar,
  Stack,
  Select,
  MenuItem,
  FormControl,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  Button,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import AddIcon from '@mui/icons-material/Add';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import SmartToyIcon from '@mui/icons-material/SmartToy';
import ReplayIcon from '@mui/icons-material/Replay';
import DeleteIcon from '@mui/icons-material/Delete';
import CircularProgress from '@mui/material/CircularProgress';
import Tooltip from '@mui/material/Tooltip';
import { MessageBubble, WelcomeMessage } from './ChatMessages';
import ChatInput from './ChatInput';

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


const ChatDrawer = ({ open, onClose }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  // Get chat state from Redux
  const activeChatId = useSelector((state) => state.chat.activeChatId);
  const activeConversation = useSelector((state) => {
    if (activeChatId) return state.chat.conversations[activeChatId];
    // Show pending messages when no active chat
    return state.chat.conversations['_pending'] || null;
  });
  const messages = activeConversation?.messages || [];
  const loading = useSelector((state) => state.chat.loading);
  const availableChats = useSelector((state) => state.chat.availableChats);

  // Local UI state
  const [showOptions, setShowOptions] = useState(true);
  const [isRecording, setIsRecording] = useState(false);
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
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

    // Handle "new" option - clear active chat to start fresh
    if (newChatId === 'new') {
      dispatch(chatActions.setActiveChat(null));
      setShowOptions(true);
      return;
    }

    if (newChatId && newChatId !== activeChatId) {
      dispatch(chatActions.setActiveChat(newChatId));
      setShowOptions(false);
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

  // Refresh available chats when activeChatId changes (e.g., when server creates a new chat)
  useEffect(() => {
    if (activeChatId && !availableChats.find((c) => c.id === activeChatId)) {
      fetchAvailableChats();
    }
  }, [activeChatId]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (messageToSend, fromAudio = false) => {
    if (!messageToSend.trim() || loading.sendingMessage) return;

    try {
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: true }));

      const chatIdForMessage = activeChatId || '_pending';
      dispatch(
        chatActions.addMessage({
          chatId: chatIdForMessage,
          message: {
            role: 'user',
            content: messageToSend,
            timestamp: new Date().toISOString(),
          },
        })
      );

      setShowOptions(false);
      sendChatMessage(activeChatId, messageToSend);

      if (fromAudio) {
        window.shouldPlayAudioResponse = true;
      }
    } catch (error) {
      console.error('Error sending message:', error);
      dispatch(chatActions.setError(error.message));

      const chatIdForError = activeChatId || '_pending';
      dispatch(
        chatActions.addMessage({
          chatId: chatIdForError,
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

  const handleDrawerClose = () => {
    onClose(false);
  };

  const clearChat = () => {
    // Clear active chat - new chat will be created when first message is sent
    dispatch(chatActions.setActiveChat(null));
    setShowOptions(true);
  };

  const handleDeleteClick = () => {
    if (!activeChatId) return;
    setDeleteDialogOpen(true);
  };

  const handleDeleteConfirm = async () => {
    setDeleteDialogOpen(false);
    try {
      const response = await fetch(`/api/chat/chats/${activeChatId}`, { method: 'DELETE' });
      if (response.ok) {
        dispatch(chatActions.setActiveChat(null));
        setShowOptions(true);
        fetchAvailableChats();
      }
    } catch (error) {
      console.error('Error deleting chat:', error);
    }
  };

  const handleDeleteCancel = () => {
    setDeleteDialogOpen(false);
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
                {activeChatId && (
                  <Tooltip title="Delete current chat">
                    <IconButton size="small" color="inherit" onClick={handleDeleteClick} sx={{ mr: 0.5 }}>
                      <DeleteIcon fontSize="small" />
                    </IconButton>
                  </Tooltip>
                )}
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
                    value={activeChatId || 'new'}
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
                    {/* Option for new chat (no activeChatId) */}
                    <MenuItem value="new">New Chat</MenuItem>
                    {/* Option for active chat not yet in available list (just created by server) */}
                    {activeChatId && !availableChats.find((c) => c.id === activeChatId) && (
                      <MenuItem value={activeChatId}>
                        {activeConversation?.name || `Chat ${activeChatId.slice(5, 13)}`}
                      </MenuItem>
                    )}
                    {availableChats.map((chat) => {
                      // Extract short ID from chat.id (e.g., "chat_1737312000000_abc123" -> "1737312000000")
                      const shortId = chat.id.replace('chat_', '').split('_')[0];
                      return (
                        <MenuItem key={chat.id} value={chat.id}>
                          {chat.name || `Chat #${shortId}`}
                        </MenuItem>
                      );
                    })}
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
                    <MessageBubble key={index} message={msg} chatId={activeChatId} />
                  ))}
                  {loading.sendingMessage && (
                    <ListItem sx={{ justifyContent: 'flex-start', py: 1 }}>
                      <Avatar sx={{ bgcolor: '#1976d2', width: 32, height: 32, mr: 1 }}>
                        <SmartToyIcon fontSize="small" />
                      </Avatar>
                      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                        <CircularProgress size={16} />
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

            <ChatInput
              onSendMessage={handleSendMessage}
              loading={loading.sendingMessage}
              showOptions={showOptions}
              isRecording={isRecording}
              onStartRecording={startRecording}
              onStopRecording={stopRecording}
              hasMessages={messages.length > 0}
            />
          </Paper>
        </Rnd>
      )}

      <Dialog open={deleteDialogOpen} onClose={handleDeleteCancel}>
        <DialogTitle>Delete Chat</DialogTitle>
        <DialogContent>
          <DialogContentText>
            Are you sure you want to delete this chat? This action cannot be undone.
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleDeleteCancel}>Cancel</Button>
          <Button onClick={handleDeleteConfirm} color="error" variant="contained">
            Delete
          </Button>
        </DialogActions>
      </Dialog>
    </>
  );
};

export default ChatDrawer;
