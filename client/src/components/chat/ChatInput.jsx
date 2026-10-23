import { useState, useRef } from 'react';
import {
  Box,
  Chip,
  CircularProgress,
  IconButton,
  Button,
  TextField,
  Tooltip,
  Typography,
} from '@mui/material';
import AttachFileIcon from '@mui/icons-material/AttachFile';
import CloseIcon from '@mui/icons-material/Close';
import MicNoneOutlinedIcon from '@mui/icons-material/MicNoneOutlined';
import SendOutlinedIcon from '@mui/icons-material/SendOutlined';
import SmartToyOutlinedIcon from '@mui/icons-material/SmartToyOutlined';
import StopIcon from '@mui/icons-material/Stop';

const iconButtonSx = { width: 30, height: 30 };

/**
 * The composer: the message box, and under it what answers (the agent and the model it runs on),
 * the attachments and the send button.
 *
 * The agent is shown, not chosen: a chat's agent is fixed when it is created (a profile for the
 * legacy engine, the eve agent for eve), and switching it halfway would need the server to accept it.
 */
const ChatInput = ({
  onSendMessage,
  loading,
  isRecording,
  onStartRecording,
  onStopRecording,
  agentInfo,
  repliesToApproval,
}) => {
  const [inputValue, setInputValue] = useState('');
  const [attachedImages, setAttachedImages] = useState([]);
  const fileInputRef = useRef(null);

  const canSend = (inputValue.trim() || attachedImages.length > 0) && !loading && !isRecording;

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

  const handleKeyDown = (event) => {
    if (event.key === 'Enter' && !event.shiftKey && !event.nativeEvent.isComposing) {
      event.preventDefault();
      handleSend();
    }
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
    <Box
      sx={{
        px: 1.5,
        pt: 1.25,
        pb: 1.5,
        borderTop: '1px solid',
        borderColor: 'divider',
        bgcolor: 'background.paper',
        display: 'flex',
        flexDirection: 'column',
        gap: 1,
        flexShrink: 0,
      }}
    >
      {attachedImages.length > 0 && (
        <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5 }}>
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
        </Box>
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

      <TextField
        fullWidth
        multiline
        minRows={1}
        maxRows={5}
        placeholder={
          repliesToApproval
            ? 'Reply, or tell the assistant what to do instead…'
            : 'Message the assistant…'
        }
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        onKeyDown={handleKeyDown}
        disabled={loading || isRecording}
        slotProps={{
          htmlInput: { 'aria-label': 'Message' },
          input: {
            sx: {
              fontSize: 14,
              py: 1.25,
              px: 1.5,
              minHeight: 44,
              alignItems: 'flex-start',
              borderRadius: '6px',
              bgcolor: 'background.paper',
              '& fieldset': { borderColor: 'grey.300' },
              '&.Mui-focused fieldset': { borderColor: 'primary.main', borderWidth: 1 },
            },
          },
        }}
      />

      <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
        <Box
          title="Chosen when the chat is created"
          sx={{
            display: 'flex',
            alignItems: 'center',
            gap: 0.75,
            minWidth: 0,
            height: 30,
            px: 1.25,
            border: 1,
            borderColor: 'primary.200',
            borderRadius: '15px',
            bgcolor: 'primary.100',
            color: 'primary.800',
            fontSize: 12.5,
          }}
        >
          <SmartToyOutlinedIcon sx={{ fontSize: 15, flexShrink: 0 }} />
          <Typography component="span" noWrap sx={{ fontSize: 'inherit', fontWeight: 500 }}>
            {agentInfo.name}
          </Typography>
          {agentInfo.model && (
            <Typography
              component="span"
              noWrap
              sx={{ fontSize: 'inherit', color: 'text.secondary' }}
            >
              {agentInfo.model}
            </Typography>
          )}
        </Box>

        <Box sx={{ flex: 1 }} />

        <Tooltip title="Attach images">
          <span>
            <IconButton
              aria-label="Attach images"
              onClick={() => fileInputRef.current?.click()}
              disabled={loading || isRecording}
              sx={{
                ...iconButtonSx,
                color: attachedImages.length > 0 ? 'primary.main' : undefined,
              }}
            >
              <AttachFileIcon sx={{ fontSize: 18 }} />
            </IconButton>
          </span>
        </Tooltip>
        <Tooltip title={isRecording ? 'Stop recording' : 'Record a voice message'}>
          <span>
            <IconButton
              aria-label={isRecording ? 'Stop recording' : 'Record a voice message'}
              onClick={isRecording ? onStopRecording : onStartRecording}
              disabled={loading}
              sx={{
                ...iconButtonSx,
                color: isRecording ? 'error.main' : undefined,
                animation: isRecording ? 'pulse 1.5s ease-in-out infinite' : 'none',
                '@keyframes pulse': {
                  '0%': { opacity: 1 },
                  '50%': { opacity: 0.45 },
                  '100%': { opacity: 1 },
                },
              }}
            >
              {isRecording ? (
                <StopIcon sx={{ fontSize: 18 }} />
              ) : (
                <MicNoneOutlinedIcon sx={{ fontSize: 18 }} />
              )}
            </IconButton>
          </span>
        </Tooltip>
        <Button
          variant="contained"
          aria-label="Send"
          onClick={handleSend}
          disabled={!canSend}
          sx={{
            minWidth: 34,
            width: 34,
            height: 30,
            minHeight: 30,
            p: 0,
            ml: 0.5,
          }}
        >
          {loading ? (
            <CircularProgress size={14} color="inherit" />
          ) : (
            <SendOutlinedIcon sx={{ fontSize: 16 }} />
          )}
        </Button>
      </Box>
    </Box>
  );
};

export default ChatInput;
