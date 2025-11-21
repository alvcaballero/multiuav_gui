import React, { useState, useRef, useEffect, use } from 'react';
import { Rnd } from 'react-rnd';
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
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import DeleteIcon from '@mui/icons-material/Delete';
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
import { styled } from '@mui/material/styles';

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
  'estas ahi?',
  '¿Qué drones están disponibles?',
  'Crea una misión para que un dron vuele 100m al norte y regrese.',
  'Mueve el gimbal del dron 30 grados hacia el este.',
  'Selecciona un dron y haz que se mantenga en vuelo estacionario a 10 metros de altura.',
];

const MessageBubble = ({ message }) => {
  const isAI = message.role === 'assistant';

  return (
    <ListItem
      sx={{
        justifyContent: isAI ? 'flex-start' : 'flex-end',
        mb: 1,
      }}
    >
      <Stack direction={isAI ? 'row' : 'row-reverse'} alignItems="flex-start" spacing={1} sx={{ maxWidth: '75%' }}>
        <Avatar sx={{ bgcolor: '#1976d2', width: 32, height: 32 }}>
          {isAI ? <SmartToyIcon fontSize="small" /> : <PersonIcon fontSize="small" />}
        </Avatar>
        <Box
          sx={{
            bgcolor: isAI ? '#f5f5f5' : '#1976d2',
            color: isAI ? '#000' : '#fff',
            borderRadius: '16px',
            p: '8px 12px',
            boxShadow: '0 1px 2px rgba(0,0,0,0.1)',
            wordBreak: 'break-word',
          }}
        >
          <ListItemText primary={message.text} />
        </Box>
      </Stack>
    </ListItem>
  );
};

const ChatDrawer = ({ open, onClose }) => {
  const { classes } = useStyles();
  const [messages, setMessages] = useState([
    { text: '¡Hola! Soy tu asistente. ¿En qué puedo ayudarte?', role: 'assistant' },
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showOptions, setShowOptions] = useState(true); // Nuevo estado para controlar la visibilidad de las opciones
  const [isRecording, setIsRecording] = useState(false);
  const [mediaRecorder, setMediaRecorder] = useState(null);
  const audioChunksRef = useRef([]);
  const audioPlayerRef = useRef(null);

  const messagesEndRef = useRef(null);
  const [panelState, setPanelState] = useState({
    width: 400, // Ancho inicial
    height: '100vh', // Altura completa
    x: window.innerWidth - 400,
    y: 0,
  });

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = (messageToSend, fromAudio = false) => {
    console.log('Enviando mensaje:', messageToSend);
    if (!messageToSend.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: messageToSend,
      role: 'user',
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setShowOptions(false); // Ocultar opciones una vez que se envía el primer mensaje

    // Capturar si este mensaje fue enviado desde audio
    const shouldPlayAudio = fromAudio;
    console.log('shouldPlayAudio:', shouldPlayAudio);

    fetch('/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ provider: 'gemini', message: userMessage.text }),
    })
      .then((res) => res.json())
      .then((data) => {
        console.log('Respuesta de la IA:', data);
        const aiResponse = {
          text: '',
          role: 'assistant',
        };
        aiResponse.text = data.content || 'Lo siento, no tengo una respuesta en este momento.';
        aiResponse.role = 'assistant';
        setMessages((prevMessages) => [...prevMessages, aiResponse]);

        // Si el mensaje anterior fue de audio, reproducir la respuesta
        if (shouldPlayAudio && aiResponse.text) {
          textToSpeech(aiResponse.text);
        }
      })
      .catch(() => {
        const aiResponse = {
          text: 'Hubo un error al comunicarse con la IA.',
          role: 'assistant',
        };
        setMessages((prevMessages) => [...prevMessages, aiResponse]);
      })
      .finally(() => {
        setIsLoading(false);
      });
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
    setShowOptions(true); // Mostrar opciones nuevamente al limpiar el chat
    setMessages([
      {
        id: 1,
        text: '¡Hola! Soy tu asistente. ¿En qué puedo ayudarte?',
        role: 'assistant',
        timestamp: new Date(),
      },
    ]);
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
        alert('El micrófono está siendo usado por otra aplicación o está deshabilitado en el sistema. Por favor, cierra otras aplicaciones que puedan estar usando el micrófono.');
      } else {
        alert('No se pudo acceder al micrófono: ' + error.message + '\n\nVerifica que:\n- El micrófono esté conectado\n- El micrófono esté habilitado en la configuración del sistema\n- No esté siendo usado por otra aplicación');
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
      const transcriptionResponse = await fetch('/api/speech-tools/stt', {
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
      const response = await fetch('/api/speech-tools/tts', {
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
        >
          <Paper className={classes.container}>
            <AppBar position="static" sx={{ backgroundColor: '#673ab7' }}>
              <Toolbar className={classes.toolbar} disableGutters>
                <Typography variant="h6" className={classes.title}>
                  {'Chat AI'}
                </Typography>
                <IconButton size="small" color="inherit" onClick={clearChat} sx={{ mr: 0.5 }}>
                  <ReplayIcon fontSize="small" />
                </IconButton>
                <IconButton size="small" color="inherit" onClick={handleDrawerClose}>
                  <ChevronRightIcon fontSize="small" />
                </IconButton>
              </Toolbar>
            </AppBar>
            {/* Messages Area */}
            <Box className={classes.MessagesArea}>
              {messages.map((msg, index) => (
                <MessageBubble key={index} message={msg} />
              ))}
              {isLoading && (
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
              <div ref={messagesEndRef} />
            </Box>

            <Box sx={{ p: 2, borderTop: '1px solid #e0e0e0', backgroundColor: '#ffffff' }}>
              {/* Optios messages */}
              {showOptions && messages.length === 1 && messages[0].role === 'assistant' && (
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
                  disabled={isLoading || isRecording}
                  variant="outlined"
                  size="small"
                  sx={{
                    '& .MuiOutlinedInput-root': {
                      borderRadius: 2,
                      bgcolor: '#f8f9fa',
                    },
                  }}
                />
                <IconButton
                  onClick={isRecording ? stopRecording : startRecording}
                  disabled={isLoading}
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
                  disabled={!inputValue.trim() || isLoading || isRecording}
                  sx={{
                    bgcolor: '#1976d2',
                    color: 'white',
                    '&:hover': { bgcolor: '#1565c0' },
                    '&:disabled': { bgcolor: '#e0e0e0' },
                    width: 40,
                    height: 40,
                  }}
                >
                  {isLoading ? <CircularProgressIcon size={20} color="inherit" /> : <SendIcon fontSize="small" />}
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
