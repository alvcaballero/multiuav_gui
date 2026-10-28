import { useState, useRef, useEffect, useCallback } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { chatActions } from '../../store';
import { sendChatMessage } from '../../common/util/sendChatMessage';
import { fetchPendingApprovals } from '../../common/util/toolApproval';

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
  // "New eve chat" was pressed and nothing was sent yet. The chat is created on the first message,
  // not on the click: `_resolveEngine` needs its row to exist before that message lands, but a
  // row made by the click alone is an empty chat that then sits in the history.
  const [pendingEveChat, setPendingEveChat] = useState(false);

  // The open chat as the server lists it (it is the source of truth for chat.metadata: engine,
  // agent, parent). Null for a brand-new chat that has no row yet.
  const activeChat = availableChats.find((c) => c.id === activeChatId) ?? null;
  // Which engine the active chat runs on — 'legacy' for a brand-new/not-yet-listed chat.
  const eveDefaultAgent = useSelector((state) => state.session.server?.eveDefaultAgent ?? null);
  const awaitingEveChat = pendingEveChat && !activeChatId;
  const activeEngine = awaitingEveChat || activeChat?.engine === 'eve' ? 'eve' : 'legacy';
  // What answers the open chat, for the composer and the assistant label: the agent, and for the
  // legacy engine the provider and model it runs on. All of it is data the client already has;
  // nothing is probed.
  const llmProvider = useSelector((state) => state.session.server?.llmProvider ?? null);
  const llmModel = useSelector((state) => state.session.server?.llmModel ?? null);
  const describeAgent = (chat) =>
    awaitingEveChat && !chat
      ? { name: eveDefaultAgent ?? 'agent', model: 'eve' }
      : chat?.engine === 'eve'
        ? { name: chat.agent ?? 'agent', model: 'eve' }
        : {
            name: chat?.agent ?? 'default',
            model: [llmProvider, llmModel].filter(Boolean).join(' · ') || null,
          };
  const agentInfo = describeAgent(activeChat);
  // A subagent is a chat of its own that points at its parent; the switcher lists the siblings.
  const parentChatId = activeChat?.parentChatId ?? null;
  const rootChatId = parentChatId ?? activeChatId;
  const parentChat = availableChats.find((c) => c.id === parentChatId) ?? null;
  const pendingApprovals = useSelector((state) => state.chat.pendingApprovals);
  const chatPendingApprovals = pendingApprovals[activeChatId] ?? EMPTY_MESSAGES;
  const resolvedApprovals = useSelector(
    (state) => state.chat.resolvedApprovals[state.chat.activeChatId] ?? EMPTY_MESSAGES,
  );
  // What the open turn is doing (only the eve engine reports it): a plan can take minutes.
  const activity = useSelector((state) => state.chat.activity[activeChatId] ?? null);

  // 'chat' is the thread; 'history' replaces it with the list of conversations.
  const [view, setView] = useState('chat');
  const [historyQuery, setHistoryQuery] = useState('');
  // Runtime state of the subagents the server still knows about (it keeps them in memory only).
  const [subagentRuntime, setSubagentRuntime] = useState([]);
  const [chatToDelete, setChatToDelete] = useState(null);
  const [showOptions, setShowOptions] = useState(true);
  const [isRecording, setIsRecording] = useState(false);
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

  const handleSelectChat = async (newChatId) => {
    setView('chat');
    setPendingEveChat(false);
    if (!newChatId || newChatId === activeChatId) return;
    dispatch(chatActions.setActiveChat(newChatId));
    setShowOptions(false);
    const existingConversation = availableChats.find((c) => c.id === newChatId);
    if (existingConversation) {
      await loadChatHistory(newChatId);
    }
  };

  const openHistory = () => {
    fetchAvailableChats();
    setView('history');
  };

  const closeHistory = () => setView('chat');

  // Back goes one level up: out of the history list, or from a subagent to the chat that spawned it.
  const handleBack = () => {
    if (view === 'history') closeHistory();
    else if (parentChatId) handleSelectChat(parentChatId);
  };

  // The server only remembers subagents while it runs; the chat list (the DB) remembers them for
  // good. The switcher shows both, so a subagent that just started is there before the list reloads.
  const refreshSubagents = useCallback(async () => {
    if (!rootChatId) {
      setSubagentRuntime([]);
      return;
    }
    try {
      const response = await fetch(`/api/chat/subagents/${rootChatId}`);
      if (response.ok) {
        const data = await response.json();
        setSubagentRuntime(data.subagents ?? []);
        // Navigation needs the persisted row (engine, endpoint display name and parent link).
        if (data.subagents?.length) await fetchAvailableChats();
      }
    } catch (error) {
      console.error('Error fetching subagents:', error);
    }
  }, [rootChatId, fetchAvailableChats]);

  const knownSubagents = new Map();
  // A chat that has no row yet has no root: matching `null` against the top-level chats' own
  // `parentChatId: null` would list every one of them as its subagent.
  for (const chat of availableChats) {
    if (rootChatId && chat.parentChatId === rootChatId) {
      knownSubagents.set(chat.id, {
        chatId: chat.id,
        agentType: chat.agent,
        status: 'done',
        createdAt: chat.createdAt,
      });
    }
  }
  for (const record of subagentRuntime) {
    knownSubagents.set(record.chatId, { ...knownSubagents.get(record.chatId), ...record });
  }
  const subagents = [...knownSubagents.values()]
    .map((subagent) => ({
      ...subagent,
      approvalsWaiting: pendingApprovals[subagent.chatId]?.length ?? 0,
    }))
    .sort((a, b) => new Date(b.createdAt) - new Date(a.createdAt));
  // The line the switcher shows for the main thread: the agent of the chat that spawned the others.
  const mainInfo = describeAgent(parentChat ?? activeChat);
  const mainMeta = [mainInfo.name, mainInfo.model].filter(Boolean).join(' · ');
  const subagentsRunning = subagents.some((subagent) => subagent.status === 'running');

  useEffect(() => {
    if (open) fetchAvailableChats();
  }, [open, fetchAvailableChats]);

  // A subagent is spawned and finishes in the middle of a turn: look again when the thread moves,
  // and keep looking while one is running (its state is the one thing that changes without a message).
  useEffect(() => {
    if (!open || view !== 'chat') return undefined;
    refreshSubagents();
    // Eve can discover its first child before any transcript message is appended.
    if (!subagentsRunning && activeEngine !== 'eve') return undefined;
    const poll = setInterval(refreshSubagents, 4000);
    return () => clearInterval(poll);
  }, [open, view, refreshSubagents, subagentsRunning, activeEngine, messages.length]);

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

    let targetChatId = activeChatId;
    try {
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: true }));
      if (awaitingEveChat) {
        targetChatId = await createEveChat();
        setPendingEveChat(false);
      }
      const chatIdForMessage = targetChatId || '_pending';
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
      sendChatMessage(targetChatId, messageToSend);
      if (fromAudio) {
        window.shouldPlayAudioResponse = true;
      }
    } catch (error) {
      console.error('Error sending message:', error);
      dispatch(chatActions.setError(error.message));
      const chatIdForError = targetChatId || '_pending';
      dispatch(
        chatActions.addMessage({
          chatId: chatIdForError,
          message: {
            role: 'assistant',
            content: awaitingEveChat
              ? `Error: ${error.message}`
              : 'Error: No se pudo enviar el mensaje. Por favor verifica la conexión WebSocket.',
            timestamp: new Date().toISOString(),
          },
        }),
      );
    } finally {
      dispatch(chatActions.setLoading({ key: 'sendingMessage', value: false }));
    }
  };

  // A new legacy chat is created lazily by the server on the first message.
  const handleNewLegacyChat = () => {
    setPendingEveChat(false);
    dispatch(chatActions.setActiveChat(null));
    setShowOptions(true);
    setView('chat');
  };

  /**
   * Creates the chat row of an eve chat. Unlike a legacy chat (the server makes it on the first WS
   * message), it has to exist before that message: `chatController._resolveEngine` only knows a
   * chat is 'eve' from its DB row.
   * @returns {Promise<string>} the new chat id, already active
   */
  const createEveChat = async () => {
    dispatch(chatActions.setLoading({ key: 'creatingChat', value: true }));
    try {
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
      return chat.id;
    } finally {
      dispatch(chatActions.setLoading({ key: 'creatingChat', value: false }));
    }
  };

  // Only marks the intent: nothing is created until the first message is sent.
  const handleNewEveChat = () => {
    dispatch(chatActions.setActiveChat(null));
    setPendingEveChat(true);
    setShowOptions(true);
    setView('chat');
  };

  const handleDeleteClick = (chatId) => {
    if (chatId) setChatToDelete(chatId);
  };

  const handleDeleteConfirm = async () => {
    const chatId = chatToDelete;
    setChatToDelete(null);
    try {
      const response = await fetch(`/api/chat/chats/${chatId}`, { method: 'DELETE' });
      if (response.ok) {
        if (chatId === activeChatId) {
          dispatch(chatActions.setActiveChat(null));
          setShowOptions(true);
        }
        fetchAvailableChats();
      }
    } catch (error) {
      console.error('Error deleting chat:', error);
    }
  };

  const handleDeleteCancel = () => setChatToDelete(null);

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
    deleteDialogOpen: chatToDelete !== null,
    messagesContainerRef,
    messagesContentRef,
    eveEnabled,
    activeEngine,
    agentInfo,
    activity,
    view,
    historyQuery,
    activeChat,
    parentChat,
    rootChatId,
    subagents,
    mainMeta,
    pendingApprovals: chatPendingApprovals,
    resolvedApprovals,
    // Handlers
    handleSendMessage,
    handleSelectChat,
    handleMessagesScroll,
    handleUserScrollIntent,
    handleNewLegacyChat,
    handleNewEveChat,
    openHistory,
    handleBack,
    setHistoryQuery,
    refreshSubagents,
    handleDeleteClick,
    handleDeleteConfirm,
    handleDeleteCancel,
    startRecording,
    stopRecording,
  };
};

export default useChatLogic;
