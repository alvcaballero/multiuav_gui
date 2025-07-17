import React, { useState, useRef, useEffect } from 'react';
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
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import DeleteIcon from '@mui/icons-material/Delete';
import ChevronLeftIcon from '@mui/icons-material/ChevronLeft';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import SendIcon from '@mui/icons-material/Send';
import PersonIcon from '@mui/icons-material/Person';
import SmartToyIcon from '@mui/icons-material/SmartToy';
import { styled } from '@mui/material/styles';

const useStyles = makeStyles()((theme) => ({
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
    marginBottom: theme.spacing(2),
    paddingRight: theme.spacing(1), // Add some padding for scrollbar
  },
  MessageInputArea: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
  },
}));
const ChatContainer = styled(Box)(({ theme }) => ({
  width: 300,
  display: 'flex',
  flexDirection: 'column',
  height: '100%',
  padding: theme.spacing(2),
  boxSizing: 'border-box',
}));

const MessagesArea = styled(List)(({ theme }) => ({
  flexGrow: 1,
  overflowY: 'auto',
  display: 'flex',
  flexDirection: 'column',
  marginBottom: theme.spacing(2),
  paddingRight: theme.spacing(1), // Add some padding for scrollbar
}));

const MessageInputArea = styled(Box)(({ theme }) => ({
  display: 'flex',
  alignItems: 'center',
  gap: theme.spacing(1),
}));

const OptionsContainer = styled(Box)(({ theme }) => ({
  display: 'flex',
  flexDirection: 'column',
  gap: theme.spacing(1),
  marginBottom: theme.spacing(2),
}));

const initialOptions = ['¿Qué puedes hacer?', 'Cuéntame un chiste', 'Ayuda con mi cuenta', 'Soporte técnico'];

const MessageBubble = ({ message }) => {
  const isAI = message.sender === 'ai';

  return (
    <ListItem
      sx={{
        display: 'flex',
        justifyContent: isAI ? 'flex-start' : 'flex-end',
        alignItems: 'flex-start',
        gap: 1,
        py: 1,
      }}
    >
      {isAI && (
        <Avatar sx={{ bgcolor: '#1976d2', width: 32, height: 32 }}>
          <SmartToyIcon fontSize="small" />
        </Avatar>
      )}

      <Box sx={{ maxWidth: '70%' }}>
        <Paper
          elevation={1}
          sx={{
            p: 1.5,
            bgcolor: isAI ? '#f5f5f5' : '#1976d2',
            color: isAI ? '#000' : '#fff',
            borderRadius: 2,
            borderTopLeftRadius: isAI ? 0.5 : 2,
            borderTopRightRadius: isAI ? 2 : 0.5,
          }}
        >
          <Typography variant="body2" sx={{ wordBreak: 'break-word' }}>
            {message.text}
          </Typography>
        </Paper>
      </Box>

      {!isAI && (
        <Avatar sx={{ bgcolor: '#4caf50', width: 32, height: 32 }}>
          <PersonIcon fontSize="small" />
        </Avatar>
      )}
    </ListItem>
  );
};

const ChatDrawer = ({ open, onClose }) => {
  const { classes } = useStyles();
  const [messages, setMessages] = useState([
    { text: '¡Hola! Soy tu asistente de IA. ¿En qué puedo ayudarte hoy?', sender: 'ai' },
  ]);
  const [newMessage, setNewMessage] = React.useState('');
  const [showOptions, setShowOptions] = React.useState(true); // Nuevo estado para controlar la visibilidad de las opciones
  const [panelWidth, setPanelWidth] = useState(300); // Estado para el ancho del chat
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = (messageToSend) => {
    if (messageToSend.trim() !== '') {
      const userMessage = { text: messageToSend.trim(), sender: 'user' };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setNewMessage('');
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
        });
      //setTimeout(() => {
      //  const aiResponse = {
      //    text: `Has dicho: "${userMessage.text}". Estoy procesando tu solicitud...`,
      //    sender: 'ai',
      //  };
      //  setMessages((prevMessages) => [...prevMessages, aiResponse]);
      //}, 1000);
    }
  };

  const handleKeyPress = (event) => {
    if (event.key === 'Enter') {
      handleSendMessage(newMessage);
    }
  };
  const handleDrawerClose = () => {
    onClose(false);
  };
  const handleOptionClick = (optionText) => {
    console.log(`Opción seleccionada: ${optionText}`);
    handleSendMessage(optionText); // Enviar la opción seleccionada como un mensaje del usuario
  };

  return (
    <>
      <Rnd
        size={{ width: panelWidth, height: window.innerHeight }}
        position={{ x: window.innerWidth - panelWidth, y: 0 }}
        minWidth={200}
        maxWidth={600}
        bounds="window"
        enableResizing={{
          left: true, // Solo permite redimensionar desde el borde izquierdo
        }}
        dragAxis="none" // No se puede mover, solo redimensionar
        onResize={(e, direction, ref, delta, position) => {
          setPanelWidth(ref.offsetWidth); // Actualiza el ancho del panel al redimensionar
        }}
        style={{
          position: 'fixed',
          top: 0,
          right: 0,
          height: '100vh',
          background: '#f5f5f5',
          boxShadow: '-2px 0 8px rgba(0,0,0,0.1)',
          zIndex: 9999,
        }}
      >
        <Toolbar className={classes.toolbar} disableGutters>
          <Typography variant="h6" className={classes.title}>
            {'Chat AI'}
          </Typography>
          <IconButton size="small" color="inherit" onClick={handleDrawerClose}>
            <ChevronRightIcon fontSize="small" />
          </IconButton>
        </Toolbar>
        <Divider />
        <MessagesArea>
          {messages.map((msg, index) => (
            <MessageBubble key={index} message={msg} />
          ))}
          <div ref={messagesEndRef} />
        </MessagesArea>
        {/* Mostrar botones de opciones solo si showOptions es true y solo hay el mensaje inicial de la IA */}
        {showOptions && messages.length === 1 && messages[0].sender === 'ai' && (
          <OptionsContainer>
            {initialOptions.map((option, index) => (
              <Button
                key={index}
                variant="outlined"
                onClick={() => handleOptionClick(option)}
                sx={{ justifyContent: 'flex-start' }} // Alinea el texto del botón a la izquierda
              >
                {option}
              </Button>
            ))}
          </OptionsContainer>
        )}
        <MessageInputArea>
          <TextField
            fullWidth
            variant="outlined"
            size="small"
            placeholder="Escribe tu mensaje..."
            value={newMessage}
            onChange={(e) => setNewMessage(e.target.value)}
            onKeyPress={handleKeyPress}
          />
          <IconButton color="primary" onClick={() => handleSendMessage(newMessage)}>
            <SendIcon />
          </IconButton>
        </MessageInputArea>
      </Rnd>
    </>
  );
};

export default ChatDrawer;
