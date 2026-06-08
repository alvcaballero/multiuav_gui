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
import AddIcon from '@mui/icons-material/Add';
import SmartToyIcon from '@mui/icons-material/SmartToy';
import ReplayIcon from '@mui/icons-material/Replay';
import DeleteIcon from '@mui/icons-material/Delete';
import CircularProgress from '@mui/material/CircularProgress';
import Tooltip from '@mui/material/Tooltip';
import { MessageBubble, WelcomeMessage } from './ChatMessages';
import ChatInput from './ChatInput';

const ChatPanel = ({
  // layout
  titleSlot,
  sx,
  // chat logic (from useChatLogic)
  activeChatId,
  activeConversation,
  messages,
  loading,
  availableChats,
  showOptions,
  isRecording,
  deleteDialogOpen,
  messagesEndRef,
  handleSendMessage,
  handleChatChange,
  clearChat,
  handleDeleteClick,
  handleDeleteConfirm,
  handleDeleteCancel,
  startRecording,
  stopRecording,
}) => (
  <Paper
    sx={{
      width: '100%',
      height: '100%',
      display: 'flex',
      flexDirection: 'column',
      overflow: 'hidden',
      borderRadius: 2,
      border: '1px solid #e0e0e0',
      ...sx,
    }}
  >
    <AppBar position="static" sx={{ backgroundColor: '#673ab7' }}>
      <Toolbar sx={{ px: 2, minHeight: 48 }} disableGutters>
        <Typography variant="h6" sx={{ flexGrow: 1 }}>
          Chat Assistant
        </Typography>
        {titleSlot}
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
      </Toolbar>
    </AppBar>

    {/* Chat selector */}
    <Box sx={{ p: 1, borderBottom: '1px solid #e0e0e0', backgroundColor: '#fafafa' }}>
      <Stack direction="row" spacing={1} sx={{ alignItems: 'center' }}>
        <FormControl size="small" sx={{ flexGrow: 1 }}>
          <Select
            value={activeChatId || 'new'}
            onChange={handleChatChange}
            displayEmpty
            disabled={loading.loadingChatList}
            sx={{ bgcolor: '#ffffff', '& .MuiSelect-select': { py: 1 } }}
          >
            <MenuItem value="new">New Chat</MenuItem>
            {activeChatId && !availableChats.find((c) => c.id === activeChatId) && (
              <MenuItem value={activeChatId}>
                {activeConversation?.name || `Chat ${activeChatId.slice(5, 13)}`}
              </MenuItem>
            )}
            {availableChats.map((chat) => {
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
            sx={{ bgcolor: '#673ab7', color: 'white', '&:hover': { bgcolor: '#5e35b1' } }}
          >
            <AddIcon fontSize="small" />
          </IconButton>
        </Tooltip>
      </Stack>
    </Box>

    {/* Messages area */}
    <Box
      sx={{
        flexGrow: 1,
        overflowY: 'auto',
        display: 'flex',
        flexDirection: 'column',
        mb: 0,
        pr: 1,
      }}
    >
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
  </Paper>
);

export default ChatPanel;
