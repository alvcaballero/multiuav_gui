export const sendChatMessage = (chatId, message) => {
  // Get the socket from the store or window
  const socket = window.websocket;

  if (!socket || socket.readyState !== WebSocket.OPEN) {
    console.error('WebSocket not connected');
    throw new Error('WebSocket not connected');
  }

  const payload = {
    type: 'chat:user_message',
    payload: {
      chatId: chatId,
      message: message,
      timestamp: new Date().toISOString(),
    },
  };

  console.log('Sending chat message via WebSocket:', payload);
  socket.send(JSON.stringify(payload));
};
