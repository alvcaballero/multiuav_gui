import { useState, useRef, useEffect, useCallback } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { chatActions } from '../../store';
import { sendChatMessage } from '../../services/sendChatMessage';

const EMPTY_MESSAGES = [];

const useChatLogic = (open = true) => {
  const dispatch = useDispatch();

  const activeChatId = useSelector((state) => state.chat.activeChatId);
  const activeConversation = useSelector((state) => {
    if (state.chat.activeChatId) return state.chat.conversations[state.chat.activeChatId];
    return state.chat.conversations['_pending'] || null;
  });
  const messages = activeConversation?.messages || EMPTY_MESSAGES;
  const hasMoreOlder = activeConversation?.hasMoreOlder ?? false;
  const loading = useSelector((state) => state.chat.loading);
  const availableChats = useSelector((state) => state.chat.availableChats);

  const [showOptions, setShowOptions] = useState(true);
  const [isRecording, setIsRecording] = useState(false);
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [mediaRecorder, setMediaRecorder] = useState(null);
  const [loadingOlderMessages, setLoadingOlderMessages] = useState(false);
  const audioChunksRef = useRef([]);
  const messagesEndRef = useRef(null);
  const messagesContainerRef = useRef(null);
  const skipAutoScrollRef = useRef(false);

  const fetchAvailableChats = useCallback(async () => {
    try {
      dispatch(chatActions.setLoading({ key: 'loadingChatList', value: true }));
      const response = await fetch('/api/chat/chats');
      if (response.ok) {
        const data = await response.json();
        dispatch(chatActions.setAvailableChats(data.chats || []));
      }
    } catch (error) {
      console.error('Error fetching chats:', error);
    } finally {
      dispatch(chatActions.setLoading({ key: 'loadingChatList', value: false }));
    }
  }, [dispatch]);

  const loadChatHistory = async (chatId) => {
    try {
      dispatch(chatActions.setLoading({ key: 'loadingHistory', value: true }));
      const response = await fetch(`/api/chat/history/${chatId}`);
      if (response.ok) {
        const data = await response.json();
        if (data.messages && data.messages.length > 0) {
          dispatch(
            chatActions.setMessages({
              chatId,
              messages: data.messages,
              hasMore: data.hasMore ?? false,
            }),
          );
          setShowOptions(false);
        }
      }
    } catch (error) {
      console.error('Error loading chat history:', error);
    } finally {
      dispatch(chatActions.setLoading({ key: 'loadingHistory', value: false }));
    }
  };

  // Fetch the page of messages just before the oldest one currently in memory
  // (server DB is the source of truth — capped/trimmed messages are still
  // there) and prepend it, preserving the user's scroll position.
  const loadOlderMessages = useCallback(async () => {
    if (!activeChatId || loadingOlderMessages || !hasMoreOlder) return;
    const oldest = activeConversation?.messages?.[0];
    if (!oldest) return;

    const container = messagesContainerRef.current;
    const previousScrollHeight = container?.scrollHeight ?? 0;
    const previousScrollTop = container?.scrollTop ?? 0;

    setLoadingOlderMessages(true);
    try {
      const response = await fetch(
        `/api/chat/history/${activeChatId}?before=${encodeURIComponent(oldest.timestamp)}&limit=50`,
      );
      if (response.ok) {
        const data = await response.json();
        skipAutoScrollRef.current = true;
        dispatch(
          chatActions.prependMessages({
            chatId: activeChatId,
            messages: data.messages || [],
            hasMore: data.hasMore ?? false,
          }),
        );
        requestAnimationFrame(() => {
          if (container) {
            container.scrollTop = container.scrollHeight - previousScrollHeight + previousScrollTop;
          }
        });
      }
    } catch (error) {
      console.error('Error loading older messages:', error);
    } finally {
      setLoadingOlderMessages(false);
    }
  }, [activeChatId, activeConversation, loadingOlderMessages, hasMoreOlder, dispatch]);

  const handleMessagesScroll = useCallback(() => {
    if (messagesContainerRef.current?.scrollTop < 80) {
      loadOlderMessages();
    }
  }, [loadOlderMessages]);

  const handleChatChange = async (event) => {
    const newChatId = event.target.value;
    if (newChatId === 'new') {
      dispatch(chatActions.setActiveChat(null));
      setShowOptions(true);
      return;
    }
    if (newChatId && newChatId !== activeChatId) {
      dispatch(chatActions.setActiveChat(newChatId));
      setShowOptions(false);
      const existingConversation = availableChats.find((c) => c.id === newChatId);
      if (existingConversation) {
        await loadChatHistory(newChatId);
      }
    }
  };

  useEffect(() => {
    if (open) fetchAvailableChats();
  }, [open, fetchAvailableChats]);

  useEffect(() => {
    if (activeChatId && !availableChats.find((c) => c.id === activeChatId)) {
      fetchAvailableChats();
    }
  }, [activeChatId, availableChats, fetchAvailableChats]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    if (skipAutoScrollRef.current) {
      skipAutoScrollRef.current = false;
      return;
    }
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (messageToSend, fromAudio = false) => {
    const isEmpty = Array.isArray(messageToSend)
      ? messageToSend.length === 0
      : !messageToSend.trim();
    if (isEmpty || loading.sendingMessage) return;

    try {
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: true }));
      const chatIdForMessage = activeChatId || '_pending';
      dispatch(
        chatActions.addMessage({
          chatId: chatIdForMessage,
          message: {
            role: 'user',
            content: messageToSend,
            timestamp: new Date().toISOString(),
          },
        }),
      );
      setShowOptions(false);
      sendChatMessage(activeChatId, messageToSend);
      if (fromAudio) {
        window.shouldPlayAudioResponse = true;
      }
    } catch (error) {
      console.error('Error sending message:', error);
      dispatch(chatActions.setError(error.message));
      const chatIdForError = activeChatId || '_pending';
      dispatch(
        chatActions.addMessage({
          chatId: chatIdForError,
          message: {
            role: 'assistant',
            content:
              'Error: No se pudo enviar el mensaje. Por favor verifica la conexión WebSocket.',
            timestamp: new Date().toISOString(),
          },
        }),
      );
    } finally {
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: false }));
    }
  };

  const clearChat = () => {
    dispatch(chatActions.setActiveChat(null));
    setShowOptions(true);
  };

  const handleDeleteClick = () => {
    if (!activeChatId) return;
    setDeleteDialogOpen(true);
  };

  const handleDeleteConfirm = async () => {
    setDeleteDialogOpen(false);
    try {
      const response = await fetch(`/api/chat/chats/${activeChatId}`, { method: 'DELETE' });
      if (response.ok) {
        dispatch(chatActions.setActiveChat(null));
        setShowOptions(true);
        fetchAvailableChats();
      }
    } catch (error) {
      console.error('Error deleting chat:', error);
    }
  };

  const handleDeleteCancel = () => setDeleteDialogOpen(false);

  const startRecording = async () => {
    try {
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        alert('Tu navegador no soporta grabación de audio. Por favor, usa un navegador moderno.');
        return;
      }
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const audioTracks = stream.getAudioTracks();
      if (audioTracks.length === 0) {
        alert(
          'No se detectó ningún micrófono. Por favor, conecta un micrófono e intenta de nuevo.',
        );
        stream.getTracks().forEach((track) => track.stop());
        return;
      }
      const audioTrack = audioTracks[0];
      if (!audioTrack.enabled) {
        alert(
          'El micrófono está deshabilitado. Por favor, habilítalo en la configuración del sistema.',
        );
        stream.getTracks().forEach((track) => track.stop());
        return;
      }
      const AudioContextClass = window.AudioContext || window['webkitAudioContext'];
      const audioContext = new AudioContextClass();
      const analyser = audioContext.createAnalyser();
      const microphone = audioContext.createMediaStreamSource(stream);
      microphone.connect(analyser);
      analyser.fftSize = 256;
      const dataArray = new Uint8Array(analyser.frequencyBinCount);
      setTimeout(() => {
        analyser.getByteFrequencyData(dataArray);
        const average = dataArray.reduce((a, b) => a + b) / dataArray.length;
        if (average < 1) {
          console.warn(
            '⚠️ No se detecta señal del micrófono. Puede estar desactivado o silenciado.',
          );
        }
      }, 500);
      const recorder = new MediaRecorder(stream);
      audioChunksRef.current = [];
      recorder.ondataavailable = (event) => {
        if (event.data.size > 0) audioChunksRef.current.push(event.data);
      };
      recorder.onstop = async () => {
        const audioBlob = new Blob(audioChunksRef.current, { type: 'audio/webm' });
        if (audioBlob.size < 100) {
          alert(
            'No se grabó ningún audio. Verifica que el micrófono esté habilitado y no silenciado.',
          );
          stream.getTracks().forEach((track) => track.stop());
          audioContext.close();
          return;
        }
        await sendAudioMessage(audioBlob);
        stream.getTracks().forEach((track) => track.stop());
        audioContext.close();
      };
      recorder.start();
      setMediaRecorder(recorder);
      setIsRecording(true);
    } catch (error) {
      console.error('Error al acceder al micrófono:', error);
      if (error.name === 'NotAllowedError' || error.name === 'PermissionDeniedError') {
        alert(
          'Permiso denegado. Por favor, permite el acceso al micrófono en la configuración del navegador.',
        );
      } else if (error.name === 'NotFoundError' || error.name === 'DevicesNotFoundError') {
        alert(
          'No se encontró ningún micrófono. Por favor, conecta un micrófono e intenta de nuevo.',
        );
      } else if (error.name === 'NotReadableError' || error.name === 'TrackStartError') {
        alert(
          'El micrófono está siendo usado por otra aplicación o está deshabilitado en el sistema.',
        );
      } else {
        alert('No se pudo acceder al micrófono: ' + error.message);
      }
    }
  };

  const stopRecording = () => {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
      mediaRecorder.stop();
      setIsRecording(false);
      setMediaRecorder(null);
    }
  };

  const sendAudioMessage = async (audioBlob) => {
    dispatch(chatActions.setLoading({ key: 'sendingMessage', value: true }));
    try {
      if (audioBlob.size < 1000) {
        alert('El audio es demasiado corto. Por favor, graba un mensaje más largo.');
        dispatch(chatActions.setLoading({ key: 'sendingMessage', value: false }));
        return;
      }
      const formData = new FormData();
      formData.append('audio', audioBlob, 'audio.webm');
      const transcriptionResponse = await fetch('/api/chat/stt', {
        method: 'POST',
        body: formData,
      });
      const responseData = await transcriptionResponse.json();
      if (!transcriptionResponse.ok) {
        if (responseData.hallucination) {
          alert(responseData.error || 'No se detectó voz clara en el audio.');
        } else {
          throw new Error(responseData.error || 'Error al transcribir el audio');
        }
        dispatch(chatActions.setLoading({ key: 'sendingMessage', value: false }));
        return;
      }
      const { text } = responseData;
      if (text && text.trim()) {
        handleSendMessage(text, true);
      } else {
        alert('No se pudo transcribir el audio. Por favor, intenta de nuevo.');
        dispatch(chatActions.setLoading({ key: 'sendingMessage', value: false }));
      }
    } catch (error) {
      console.error('Error al procesar el audio:', error);
      alert(
        'Hubo un error al procesar el audio. Asegúrate de hablar claramente y cerca del micrófono.',
      );
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: false }));
    }
  };

  return {
    // State
    activeChatId,
    activeConversation,
    messages,
    hasMoreOlder,
    loadingOlderMessages,
    loading,
    availableChats,
    showOptions,
    isRecording,
    deleteDialogOpen,
    messagesEndRef,
    messagesContainerRef,
    // Handlers
    handleSendMessage,
    handleChatChange,
    handleMessagesScroll,
    clearChat,
    handleDeleteClick,
    handleDeleteConfirm,
    handleDeleteCancel,
    startRecording,
    stopRecording,
  };
};

export default useChatLogic;
