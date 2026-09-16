import { Box } from '@mui/material';
import useChatLogic from '../components/chat/useChatLogic';
import ChatPanel from '../components/chat/ChatPanel';

const ChatPage = () => {
  const chatLogic = useChatLogic(true);

  return (
    <Box sx={{ height: '100vh', display: 'flex', flexDirection: 'column' }}>
      <ChatPanel {...chatLogic} />
    </Box>
  );
};

export default ChatPage;
