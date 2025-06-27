import React, { useState, useRef, useEffect } from 'react';
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
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import DeleteIcon from '@mui/icons-material/Delete';
import ChevronLeftIcon from '@mui/icons-material/ChevronLeft';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import SendIcon from '@mui/icons-material/Send';
import { styled } from '@mui/material/styles';

const useStyles = makeStyles()((theme) => ({
  drawer: {
    width: theme.dimensions.eventsDrawerWidth,
  },
  toolbar: {
    paddingLeft: theme.spacing(2),
    paddingRight: theme.spacing(2),
  },
  title: {
    flexGrow: 1,
  },
  ChatContainer: {
    width: 300,
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

const ChatDrawer = ({ open, onClose }) => {
  const { classes } = useStyles();
  const [messages, setMessages] = useState([
    { text: '¡Hola! Soy tu asistente de IA. ¿En qué puedo ayudarte hoy?', sender: 'ai' },
  ]);
  const [newMessage, setNewMessage] = React.useState('');
  const [showOptions, setShowOptions] = React.useState(true); // Nuevo estado para controlar la visibilidad de las opciones

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

      // Simular respuesta de la IA
      setTimeout(() => {
        const aiResponse = {
          text: `Has dicho: "${userMessage.text}". Estoy procesando tu solicitud...`,
          sender: 'ai',
        };
        setMessages((prevMessages) => [...prevMessages, aiResponse]);
      }, 1000);
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
    <Drawer anchor="right" open={open} variant="persistent">
      <ChatContainer>
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
            <ListItem
              key={index}
              sx={{
                justifyContent: msg.sender === 'user' ? 'flex-end' : 'flex-start',
                textAlign: msg.sender === 'user' ? 'right' : 'left',
                mb: 1,
              }}
            >
              <ListItemText
                primary={msg.text}
                sx={{
                  backgroundColor: msg.sender === 'user' ? 'primary.light' : 'grey.300',
                  color: msg.sender === 'user' ? 'white' : 'text.primary',
                  borderRadius: 2,
                  p: 1,
                  maxWidth: '80%',
                }}
              />
            </ListItem>
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
      </ChatContainer>
    </Drawer>
  );
};

export default ChatDrawer;
