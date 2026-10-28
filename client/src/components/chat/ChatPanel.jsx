import { useState } from 'react';
import {
  Box,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  ThemeProvider,
} from '@mui/material';
import ChatHistoryView from './ChatHistoryView';
import ChatInput from './ChatInput';
import { ChatNavigationContext } from './ChatNavigationContext';
import ChatThread from './ChatThread';
import ChatTitleBar from './ChatTitleBar';
import SubagentMenu, { SubagentStrip, subagentName } from './SubagentMenu';
import { chatTheme } from './chatTheme';
import { formatTimestamp } from '../../common/util/chatMessage';

/** The element that actually scrolls. */
const messagesScrollerSx = {
  flexGrow: 1,
  minHeight: 0,
  overflowY: 'auto',
  display: 'flex',
  flexDirection: 'column',
};

/**
 * The scroller's single child, sized by the transcript inside it.
 * `flexGrow` keeps it filling an empty chat; `flexShrink: 0` stops the flex layout from
 * compressing it back to the viewport height, which would leave nothing to scroll.
 */
const messagesContentSx = {
  display: 'flex',
  flexDirection: 'column',
  flexGrow: 1,
  flexShrink: 0,
  gap: 1.75,
  px: 1.75,
  py: 2,
};

/** The assistant panel (title bar, thread or history, composer) in the look of the handoff. */
const ChatPanel = ({
  // layout
  onClose,
  sx,
  // chat logic (from useChatLogic)
  activeChatId,
  activeChat,
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
  agentInfo,
  activity,
  processing,
  stopping,
  view,
  historyQuery,
  parentChat,
  rootChatId,
  subagents,
  mainMeta,
  pendingApprovals,
  resolvedApprovals,
  handleSendMessage,
  handleStopMessage,
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
}) => {
  const [subMenuAnchor, setSubMenuAnchor] = useState(null);

  const activeSubagent = subagents.find((subagent) => subagent.chatId === activeChatId) ?? null;
  const isSubagent = Boolean(activeChat?.parentChatId);
  const inHistory = view === 'history';

  const title =
    inHistory || !isSubagent
      ? 'Assistant'
      : subagentName(activeSubagent ?? { agentType: activeChat.agent });
  const subtitle = inHistory
    ? 'Chat history'
    : isSubagent
      ? `Subagent of ${parentChat?.name ?? 'Main'}`
      : (activeChat?.name ?? activeConversation?.name ?? 'New chat');

  const assistantLabel = [agentInfo.name, agentInfo.model].filter(Boolean).join(' · ');
  const working = loading.sendingMessage || processing || Boolean(activity);
  const repliesToApproval = activeEngine === 'eve' && pendingApprovals.length > 0;

  const openSubagentMenu = (event) => {
    refreshSubagents();
    setSubMenuAnchor(event.currentTarget);
  };

  const selectFromMenu = (chatId) => {
    setSubMenuAnchor(null);
    handleSelectChat(chatId);
  };

  return (
    <ThemeProvider theme={chatTheme}>
      <ChatNavigationContext value={{ openChat: handleSelectChat }}>
        <Box
          sx={{
            width: '100%',
            height: '100%',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            bgcolor: 'background.default',
            color: 'text.primary',
            ...sx,
          }}
        >
          <ChatTitleBar
            title={title}
            subtitle={subtitle}
            showBack={inHistory || isSubagent}
            subagentCount={subagents.length}
            subagentRunning={subagents.some((subagent) => subagent.status === 'running')}
            subagentMenuOpen={Boolean(subMenuAnchor)}
            eveEnabled={eveEnabled}
            onBack={handleBack}
            onToggleSubagents={openSubagentMenu}
            onHistory={openHistory}
            onNewLegacyChat={handleNewLegacyChat}
            onNewEveChat={handleNewEveChat}
            onClose={onClose}
          />

          <SubagentMenu
            anchorEl={subMenuAnchor}
            onClose={() => setSubMenuAnchor(null)}
            subagents={subagents}
            mainMeta={mainMeta}
            activeChatId={activeChatId}
            rootChatId={rootChatId}
            onSelect={selectFromMenu}
          />

          {inHistory ? (
            <ChatHistoryView
              chats={availableChats}
              query={historyQuery}
              onQueryChange={setHistoryQuery}
              activeChatId={activeChatId}
              loading={loading.loadingChatList}
              onOpen={handleSelectChat}
              onDelete={handleDeleteClick}
            />
          ) : (
            <>
              {isSubagent && (
                <SubagentStrip
                  subagent={activeSubagent}
                  meta={[
                    activeSubagent && formatTimestamp(activeSubagent.createdAt),
                    agentInfo.name,
                    agentInfo.model,
                  ]
                    .filter(Boolean)
                    .join(' · ')}
                  onGoToMain={() => handleSelectChat(parentChat?.id ?? rootChatId)}
                />
              )}

              <Box
                ref={messagesContainerRef}
                onScroll={handleMessagesScroll}
                // Gestures, so the scroll handler can tell the user moving the view apart
                // from the browser re-clamping it. onPointerDown covers dragging the
                // scrollbar itself, which emits no wheel or touch event.
                onWheel={handleUserScrollIntent}
                onTouchMove={handleUserScrollIntent}
                onPointerDown={handleUserScrollIntent}
                sx={messagesScrollerSx}
              >
                {/* Wraps everything that scrolls so a ResizeObserver can watch the content
                    box grow — that is what keeps the view pinned to the newest message
                    while rows, images and markdown settle into their real heights. */}
                <Box ref={messagesContentRef} sx={messagesContentSx}>
                  <ChatThread
                    chatId={activeChatId}
                    messages={messages}
                    pendingApprovals={pendingApprovals}
                    resolvedApprovals={resolvedApprovals}
                    isSubagent={isSubagent}
                    assistantLabel={assistantLabel}
                    working={working}
                    activity={activity}
                    hasMoreOlder={hasMoreOlder}
                    loadingOlderMessages={loadingOlderMessages}
                    showSuggestions={showOptions && messages.length === 0}
                    onSend={handleSendMessage}
                  />
                </Box>
              </Box>

              <ChatInput
                onSendMessage={handleSendMessage}
                onStopMessage={handleStopMessage}
                processing={processing}
                canStop={Boolean(activeChatId) && (processing || pendingApprovals.length > 0)}
                stopping={stopping}
                loading={loading.sendingMessage}
                isRecording={isRecording}
                onStartRecording={startRecording}
                onStopRecording={stopRecording}
                agentInfo={agentInfo}
                repliesToApproval={repliesToApproval}
              />
            </>
          )}

          <Dialog open={deleteDialogOpen} onClose={handleDeleteCancel}>
            <DialogTitle>Delete chat</DialogTitle>
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
        </Box>
      </ChatNavigationContext>
    </ThemeProvider>
  );
};

export default ChatPanel;
