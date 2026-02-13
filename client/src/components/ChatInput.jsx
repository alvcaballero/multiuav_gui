import { useState } from 'react';
import {
  IconButton,
  TextField,
  Button,
  Box,
  Stack,
} from '@mui/material';
import SendIcon from '@mui/icons-material/Send';
import MicIcon from '@mui/icons-material/Mic';
import StopIcon from '@mui/icons-material/Stop';
import CircularProgressIcon from '@mui/material/CircularProgress';

const initialOptions = [
  'Create a quick mission to inspect the offshore wind turbines in Viana do Castelo',
  'Create a mission to inspect the line B wind turbines in Viana do Castelo',
  'Create a detailed mission for wind turbine 2 of line B in Viana do Castelo',
  'Which drones are available?',
  'Create a mission for a drone to fly 100m north and return.',
  'Select a drone and make it hover at a height of 10 meters.',
];

const ChatInput = ({ onSendMessage, loading, showOptions, isRecording, onStartRecording, onStopRecording, hasMessages }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSend = () => {
    if (!inputValue.trim()) return;
    onSendMessage(inputValue);
    setInputValue('');
  };

  const handleKeyPress = (event) => {
    if (event.key === 'Enter' && !event.shiftKey) {
      event.preventDefault();
      handleSend();
    }
  };

  const handleOptionClick = (optionText) => {
    onSendMessage(optionText);
  };

  return (
    <Box sx={{ p: 2, borderTop: '1px solid #e0e0e0', backgroundColor: '#ffffff' }}>
      {showOptions && !hasMessages && (
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

      <Stack direction="row" spacing={1}>
        <TextField
          fullWidth
          multiline
          maxRows={3}
          placeholder="Ask me anything about drones, missions, or commands..."
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyUp={handleKeyPress}
          disabled={loading || isRecording}
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
          onClick={isRecording ? onStopRecording : onStartRecording}
          disabled={loading}
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
          onClick={handleSend}
          disabled={!inputValue.trim() || loading || isRecording}
          sx={{
            bgcolor: '#1976d2',
            color: 'white',
            '&:hover': { bgcolor: '#1565c0' },
            '&:disabled': { bgcolor: '#e0e0e0' },
            width: 40,
            height: 40,
          }}
        >
          {loading ? (
            <CircularProgressIcon size={20} color="inherit" />
          ) : (
            <SendIcon fontSize="small" />
          )}
        </IconButton>
      </Stack>
    </Box>
  );
};

export default ChatInput;
