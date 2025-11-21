import React, { useState, useRef, useEffect } from 'react';
import {
  IconButton,
  List,
  ListItem,
  ListItemText,
  Toolbar,
  Typography,
  Box,
  TextField,
  Button,
  Avatar,
  Paper,
  AppBar,
  Stack,
  Container,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import SendIcon from '@mui/icons-material/Send';
import PersonIcon from '@mui/icons-material/Person';
import SmartToyIcon from '@mui/icons-material/SmartToy';
import ReplayIcon from '@mui/icons-material/Replay';
import MicIcon from '@mui/icons-material/Mic';
import StopIcon from '@mui/icons-material/Stop';
import CircularProgressIcon from '@mui/material/CircularProgress';

const useStyles = makeStyles()((theme) => ({
  pageContainer: {
    height: '100vh',
    display: 'flex',
    flexDirection: 'column',
    overflow: 'hidden',
  },
  container: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
    overflow: 'hidden',
  },
  toolbar: {
    paddingLeft: theme.spacing(2),
    paddingRight: theme.spacing(2),
  },
  title: {
    flexGrow: 1,
  },
  MessagesArea: {
    flexGrow: 1,
    overflowY: 'auto',
    display: 'flex',
    flexDirection: 'column',
    padding: theme.spacing(3),
    backgroundColor: '#f5f5f5',
  },
  inputContainer: {
    padding: theme.spacing(2),
    borderTop: '1px solid #e0e0e0',
    backgroundColor: '#ffffff',
  },
}));

const initialOptions = [
  'Enviar el robot al almacen  de suministros',
  'AGV regresa a la base',
  'donde se encuentra el robot?',
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

const ChatPage = () => {
  const { classes } = useStyles();
  const [messages, setMessages] = useState([
    { text: 'Hola! Soy tu asistente. ¿En qué puedo ayudarte?', role: 'assistant' },
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showOptions, setShowOptions] = useState(true);
  const [isRecording, setIsRecording] = useState(false);
  const [mediaRecorder, setMediaRecorder] = useState(null);
  const audioChunksRef = useRef([]);
  const audioPlayerRef = useRef(null);
  const messagesEndRef = useRef(null);

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
    setShowOptions(false);

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

  const handleOptionClick = (optionText) => {
    console.log(` ${optionText}`);
    handleSendMessage(optionText);
  };

  const clearChat = () => {
    setShowOptions(true);
    setMessages([
      {
        id: 1,
        text: '�Hola! Soy tu asistente. �En qu� puedo ayudarte?',
        role: 'assistant',
        timestamp: new Date(),
      },
    ]);
  };

  const startRecording = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const recorder = new MediaRecorder(stream);
      audioChunksRef.current = [];

      recorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          audioChunksRef.current.push(event.data);
        }
      };

      recorder.onstop = async () => {
        const audioBlob = new Blob(audioChunksRef.current, { type: 'audio/webm' });
        await sendAudioMessage(audioBlob);

        stream.getTracks().forEach((track) => track.stop());
      };

      recorder.start();
      setMediaRecorder(recorder);
      setIsRecording(true);
    } catch (error) {
      console.error('Error al acceder al micr�fono:', error);
      alert('No se pudo acceder al micr�fono. Por favor, verifica los permisos.');
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
      const formData = new FormData();
      formData.append('audio', audioBlob, 'audio.webm');

      const transcriptionResponse = await fetch('/api/speech-tools/stt', {
        method: 'POST',
        body: formData,
      });

      if (!transcriptionResponse.ok) {
        throw new Error('Error al transcribir el audio');
      }

      const { text } = await transcriptionResponse.json();

      if (text && text.trim()) {
        handleSendMessage(text, true);
      } else {
        alert('No se pudo transcribir el audio. Por favor, intenta de nuevo.');
        setIsLoading(false);
      }
    } catch (error) {
      console.error('Error al procesar el audio:', error);
      alert('Hubo un error al procesar el audio.');
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

      if (audioPlayerRef.current) {
        audioPlayerRef.current.pause();
        audioPlayerRef.current = null;
      }

      const audio = new Audio(audioUrl);
      audioPlayerRef.current = audio;

      audio.play().catch((error) => {
        console.error('Error al reproducir audio:', error);
      });

      audio.onended = () => {
        URL.revokeObjectURL(audioUrl);
        audioPlayerRef.current = null;
      };
    } catch (error) {
      console.error('Error en text-to-speech:', error);
    }
  };

  return (
    <Box className={classes.pageContainer}>
      <Paper className={classes.container} elevation={0}>
        <AppBar position="static" sx={{ backgroundColor: '#673ab7' }}>
          <Toolbar className={classes.toolbar}>
            <Typography variant="h6" className={classes.title}>
              Chat AI
            </Typography>
            <IconButton size="small" color="inherit" onClick={clearChat} sx={{ mr: 0.5 }}>
              <ReplayIcon fontSize="small" />
            </IconButton>
          </Toolbar>
        </AppBar>

        {/* Messages Area */}
        <Box className={classes.MessagesArea}>
          <Container maxWidth="lg">
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
          </Container>
        </Box>

        {/* Input Area */}
        <Box className={classes.inputContainer}>
          <Container maxWidth="lg">
            {/* Options messages */}
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
          </Container>
        </Box>
      </Paper>
    </Box>
  );
};

export default ChatPage;
