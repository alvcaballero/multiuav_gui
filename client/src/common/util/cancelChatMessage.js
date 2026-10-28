export const cancelChatMessage = async (chatId) => {
  const response = await fetch(`/api/chat/chats/${encodeURIComponent(chatId)}/cancel`, {
    method: 'POST',
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || 'Failed to stop processing.');
  return result;
};
