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

const ChatDrawer = ({ open, onClose }) => {
  const { classes } = useStyles();
  const [messages, setMessages] = useState([
    { text: '¡Hola! Soy tu asistente de IA. ¿En qué puedo ayudarte hoy?', sender: 'ai' },
  ]);
  const [newMessage, setNewMessage] = React.useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = () => {
    if (newMessage.trim() !== '') {
      const userMessage = { text: newMessage.trim(), sender: 'user' };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setNewMessage('');

      // Simulate AI response
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
      handleSendMessage();
    }
  };
  const handleDrawerClose = () => {
    onClose(false);
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
          <IconButton color="primary" onClick={handleSendMessage}>
            <SendIcon />
          </IconButton>
        </MessageInputArea>
      </ChatContainer>
    </Drawer>
  );
};

export default ChatDrawer;
