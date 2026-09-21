import { useState, useRef, useEffect, useCallback } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { chatActions } from '../../store';
import { sendChatMessage } from '../../services/sendChatMessage';
import { fetchPendingApprovals } from '../../services/toolApproval';

const EMPTY_MESSAGES = [];

/** How close to the bottom still counts as "following the conversation", in px. */
const STICK_TO_BOTTOM_PX = 120;

/**
 * How long after a gesture its trailing scroll events still count as the user's.
 * Covers wheel momentum and the frames between a drag and the scroll it produces.
 */
const USER_SCROLL_INTENT_MS = 400;

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
  // Whether the eve agent orchestrator (gcs_eevee_assistant) is reachable at all —
  // gates offering it in the UI, mirrors the server-side EveEnable check in
  // chatController.createChat.
  const eveEnabled = useSelector((state) => state.session.server?.eveEnabled ?? false);
  // Which engine the active chat runs on. Looked up from availableChats (server
  // is the source of truth for chat.metadata.engine) rather than duplicated into
  // Redux conversation state — 'legacy' for a brand-new/not-yet-listed chat.
  const activeEngine =
    availableChats.find((c) => c.id === activeChatId)?.engine === 'eve' ? 'eve' : 'legacy';

  const [showOptions, setShowOptions] = useState(true);
  const [isRecording, setIsRecording] = useState(false);
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [mediaRecorder, setMediaRecorder] = useState(null);
  const [loadingOlderMessages, setLoadingOlderMessages] = useState(false);
  const audioChunksRef = useRef([]);
  const messagesContainerRef = useRef(null);
  const messagesContentRef = useRef(null);
  const skipAutoScrollRef = useRef(false);
  const isAtBottomRef = useRef(true);
  const settleFrameRef = useRef(0);
  const isSettlingRef = useRef(false);
  const userScrollIntentRef = useRef(false);
  const intentTimerRef = useRef(0);

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

    // A tool call parked on an approval lives in the server DB, so re-reading it
    // here is what makes a pending approval survive a reload — and stay
    // answerable, because the turn is still parked waiting for that requestId.
    try {
      dispatch(
        chatActions.setPendingApprovals({ chatId, requests: await fetchPendingApprovals(chatId) }),
      );
    } catch (error) {
      console.error('Error loading pending approvals:', error);
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
        // Reaching this point means the user scrolled up to the very top, so they
        // are reading history, not following the tail: unpin explicitly, or the
        // ResizeObserver would read the prepended page as growth worth chasing
        // and yank them to the newest message they just scrolled away from.
        skipAutoScrollRef.current = true;
        isAtBottomRef.current = false;
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

  /**
   * Records that the scrolling about to happen is the user's doing.
   *
   * A `scroll` event does not say who caused it, and most of the ones this
   * container sees are not the user: swapping in another chat's transcript makes
   * the browser re-clamp scrollTop, and rows rendering out of their
   * `contain-intrinsic-size` estimates move it again. Those land BEFORE the
   * ResizeObserver that re-pins — per the rendering steps, scroll events fire,
   * then rAF, then resize observations — so reading them as intent is exactly
   * what leaves a freshly opened chat parked in the middle of itself.
   *
   * The gesture events below do say who caused it. Everything else is layout.
   */
  const handleUserScrollIntent = useCallback(() => {
    userScrollIntentRef.current = true;
    clearTimeout(intentTimerRef.current);
    intentTimerRef.current = setTimeout(() => {
      userScrollIntentRef.current = false;
    }, USER_SCROLL_INTENT_MS);
  }, []);

  useEffect(() => () => clearTimeout(intentTimerRef.current), []);

  const handleMessagesScroll = useCallback(() => {
    const container = messagesContainerRef.current;
    if (!container) return;

    // Only a real gesture moves the view off the newest message — and never one
    // of `scrollToBottom`'s own scrolls, which are this component's, not a mind
    // being changed.
    if (userScrollIntentRef.current && !isSettlingRef.current) {
      const distanceFromBottom =
        container.scrollHeight - container.scrollTop - container.clientHeight;
      isAtBottomRef.current = distanceFromBottom <= STICK_TO_BOTTOM_PX;
    }

    if (container.scrollTop < 80) {
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

  // Jumping straight to the bottom instead of animating there. A turn with a tool
  // loop appends reasoning + tool_call + tool_result per iteration, so a smooth
  // scrollIntoView per message queues a dozen overlapping animations, each of
  // which forces layout on the whole transcript container every frame.
  //
  // The scroll events this emits are ours, not the user changing their mind, so
  // it flags them for `handleMessagesScroll` to ignore until the next frame.
  const scrollToBottom = useCallback(() => {
    const container = messagesContainerRef.current;
    if (!container) return;

    isSettlingRef.current = true;
    container.scrollTop = container.scrollHeight;

    cancelAnimationFrame(settleFrameRef.current);
    settleFrameRef.current = requestAnimationFrame(() => {
      isSettlingRef.current = false;
    });
  }, []);

  // Opening a transcript lands on the newest message, the way every chat behaves,
  // and staying there is not a single scroll: the container grows for a while
  // after the messages are committed. Rows are `content-visibility: auto`, so each
  // one's real height only replaces its `contain-intrinsic-size` estimate once the
  // browser actually renders it, and images, markdown and fonts settle later still
  // — every one of those pushes the bottom further down. Chasing that with a fixed
  // number of frames is a guess that a 300-message history loses. Watching the
  // content box instead re-pins on each growth step, however long they take, and
  // costs nothing while the transcript is idle.
  useEffect(() => {
    const content = messagesContentRef.current;
    if (!content || typeof ResizeObserver === 'undefined') return undefined;

    // Setting scrollTop never resizes anything, so this cannot feed itself.
    const observer = new ResizeObserver(() => {
      if (isAtBottomRef.current) scrollToBottom();
    });

    observer.observe(content);
    return () => {
      observer.disconnect();
      cancelAnimationFrame(settleFrameRef.current);
    };
  }, [scrollToBottom]);

  // Selecting another chat always opens it at the end, whatever the previous one
  // was scrolled to. Runs before the effect below, so the messages that arrive
  // with the switch are pinned rather than left wherever the old chat sat.
  useEffect(() => {
    isAtBottomRef.current = true;
  }, [activeChatId]);

  useEffect(() => {
    if (skipAutoScrollRef.current) {
      skipAutoScrollRef.current = false;
      return;
    }
    if (!isAtBottomRef.current) return;
    scrollToBottom();
  }, [messages, scrollToBottom]);

  const handleSendMessage = async (messageToSend, fromAudio = false) => {
    const isEmpty = Array.isArray(messageToSend)
      ? messageToSend.length === 0
      : !messageToSend.trim();
    if (isEmpty || loading.sendingMessage) return;

    // Sending is an explicit "I want to see what happens next" — re-pin to the
    // bottom even if the user had scrolled up to read something older.
    isAtBottomRef.current = true;

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

  /**
   * Starts a new chat on the eve agent orchestrator (`gcs_eevee_assistant`)
   * instead of the legacy MessageOrchestrator. Unlike a legacy "new chat"
   * (created lazily on the first WS message), this chat is created explicitly
   * up front — `chatController._resolveEngine` only knows a chat is 'eve' from
   * its DB row, so it has to exist before the first `chat:user_message` for
   * that chatId lands.
   */
  const handleNewEveChat = async () => {
    try {
      dispatch(chatActions.setLoading({ key: 'creatingChat', value: true }));
      const response = await fetch('/api/chat/chats', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ engine: 'eve', name: 'Eve chat' }),
      });
      if (!response.ok) {
        const err = await response.json().catch(() => ({}));
        throw new Error(err.error || 'Failed to create eve chat');
      }
      const chat = await response.json();
      // Refresh BEFORE switching so activeEngine (looked up from availableChats)
      // reads 'eve' on the very first render of this chat, not one tick later.
      await fetchAvailableChats();
      dispatch(chatActions.setActiveChat(chat.id));
      setShowOptions(false);
    } catch (error) {
      console.error('Error creating eve chat:', error);
      dispatch(chatActions.setError(error.message));
    } finally {
      dispatch(chatActions.setLoading({ key: 'creatingChat', value: false }));
    }
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
    messagesContainerRef,
    messagesContentRef,
    eveEnabled,
    activeEngine,
    // Handlers
    handleSendMessage,
    handleChatChange,
    handleMessagesScroll,
    handleUserScrollIntent,
    clearChat,
    handleNewEveChat,
    handleDeleteClick,
    handleDeleteConfirm,
    handleDeleteCancel,
    startRecording,
    stopRecording,
  };
};

export default useChatLogic;
