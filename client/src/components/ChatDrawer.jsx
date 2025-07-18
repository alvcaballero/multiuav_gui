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
  const isAI = message.sender === 'ai';

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
    { text: '¡Hola! Soy tu asistente. ¿En qué puedo ayudarte?', sender: 'ai' },
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showOptions, setShowOptions] = useState(true); // Nuevo estado para controlar la visibilidad de las opciones

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

  const handleSendMessage = (messageToSend) => {
    console.log('Enviando mensaje:', messageToSend);
    if (!messageToSend.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: messageToSend,
      sender: 'user',
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setShowOptions(false); // Ocultar opciones una vez que se envía el primer mensaje

    fetch('/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ provider: 'gemini', message: userMessage.text }),
    })
      .then((res) => res.json())
      .then((data) => {
        const aiResponse = {
          text: data.reply || 'No se recibió respuesta de la IA.',
          sender: 'ai',
        };
        setMessages((prevMessages) => [...prevMessages, aiResponse]);
      })
      .catch(() => {
        const aiResponse = {
          text: 'Hubo un error al comunicarse con la IA.',
          sender: 'ai',
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
        sender: 'ai',
        timestamp: new Date(),
      },
    ]);
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
              {showOptions && messages.length === 1 && messages[0].sender === 'ai' && (
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
                  disabled={isLoading}
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
                  onClick={() => handleSendMessage(inputValue)}
                  disabled={!inputValue.trim() || isLoading}
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
