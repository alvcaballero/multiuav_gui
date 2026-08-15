import { useState, useRef } from 'react';
import { IconButton, TextField, Button, Box, Stack, Chip } from '@mui/material';
import SendIcon from '@mui/icons-material/Send';
import MicIcon from '@mui/icons-material/Mic';
import StopIcon from '@mui/icons-material/Stop';
import AttachFileIcon from '@mui/icons-material/AttachFile';
import CloseIcon from '@mui/icons-material/Close';
import CircularProgressIcon from '@mui/material/CircularProgress';

const initialOptions = [
  'Which drones are available?',
  `Inspect the entire wind farm using all available UAVs to get a general assessment of its condition`,
  `Inspecciona la línea A y la línea B de la granja de aerogeneradores.`,
];

const ChatInput = ({
  onSendMessage,
  loading,
  showOptions,
  isRecording,
  onStartRecording,
  onStopRecording,
  hasMessages,
}) => {
  const [inputValue, setInputValue] = useState('');
  const [attachedImages, setAttachedImages] = useState([]);
  const fileInputRef = useRef(null);

  const handleSend = () => {
    if (!inputValue.trim() && attachedImages.length === 0) return;

    if (attachedImages.length > 0) {
      const content = [
        { type: 'input_text', text: inputValue.trim() || ' ' },
        ...attachedImages.map((img) => ({
          type: 'input_image',
          image_url: img.dataUrl,
        })),
      ];
      onSendMessage(content);
    } else {
      onSendMessage(inputValue);
    }

    setInputValue('');
    setAttachedImages([]);
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

  const handleFileChange = (event) => {
    const files = Array.from(event.target.files);
    files.forEach((file) => {
      const reader = new FileReader();
      reader.onload = (e) => {
        setAttachedImages((prev) => [...prev, { name: file.name, dataUrl: e.target.result }]);
      };
      reader.readAsDataURL(file);
    });
    event.target.value = '';
  };

  const removeImage = (index) => {
    setAttachedImages((prev) => prev.filter((_, i) => i !== index));
  };

  return (
    <Box sx={{ p: 2, borderTop: '1px solid #e0e0e0', backgroundColor: '#ffffff' }}>
      {showOptions && !hasMessages && (
        <Box sx={{ mb: 2 }}>
          <Stack direction="column" sx={{ flexWrap: 'wrap' }} spacing={1} useFlexGap>
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

      {attachedImages.length > 0 && (
        <Stack direction="row" spacing={0.5} useFlexGap sx={{ flexWrap: 'wrap', mb: 1 }}>
          {attachedImages.map((img, i) => (
            <Chip
              key={img.dataUrl}
              label={img.name}
              size="small"
              onDelete={() => removeImage(i)}
              deleteIcon={<CloseIcon />}
              sx={{ maxWidth: 160 }}
            />
          ))}
        </Stack>
      )}

      <input
        ref={fileInputRef}
        type="file"
        accept="image/*"
        multiple
        aria-label="Attach images"
        style={{ display: 'none' }}
        onChange={handleFileChange}
      />

      <Stack direction="row" spacing={1}>
        <IconButton
          aria-label="Attach images"
          onClick={() => fileInputRef.current?.click()}
          disabled={loading || isRecording}
          sx={{
            bgcolor: '#f5f5f5',
            color: attachedImages.length > 0 ? '#1976d2' : '#757575',
            '&:hover': { bgcolor: '#e0e0e0' },
            '&:disabled': { bgcolor: '#f5f5f5' },
            width: 40,
            height: 40,
          }}
        >
          <AttachFileIcon fontSize="small" />
        </IconButton>
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
          disabled={(!inputValue.trim() && attachedImages.length === 0) || loading || isRecording}
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
