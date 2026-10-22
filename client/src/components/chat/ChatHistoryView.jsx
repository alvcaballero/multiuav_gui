import {
  Box,
  ButtonBase,
  IconButton,
  InputAdornment,
  TextField,
  Tooltip,
  Typography,
} from '@mui/material';
import DeleteOutlineIcon from '@mui/icons-material/DeleteOutlineOutlined';
import SearchIcon from '@mui/icons-material/Search';
import StatusTag from './StatusTag';
import { chatColors, kickerSx } from './chatTheme';
import { groupHistory, historyTimeLabel } from '../../common/util/chatHistory';

const SubagentCountTag = ({ count }) => (
  <Box
    component="span"
    sx={{
      display: 'inline-flex',
      alignItems: 'center',
      height: 22,
      px: 1,
      fontSize: 12,
      fontWeight: 500,
      color: chatColors.accent[700],
      border: `1px solid ${chatColors.accent[400]}`,
    }}
  >
    {count === 1 ? '1 subagent' : `${count} subagents`}
  </Box>
);

const HistoryItem = ({ chat, active, onOpen, onDelete }) => (
  <Box
    sx={{
      position: 'relative',
      borderLeft: '2px solid',
      borderColor: active ? 'primary.main' : 'transparent',
      bgcolor: active ? chatColors.accent[100] : 'transparent',
      '&:hover': { bgcolor: active ? chatColors.accent[100] : 'rgba(89, 128, 166, 0.08)' },
      '&:hover .delete, &:focus-within .delete': { opacity: 1 },
    }}
  >
    <ButtonBase
      onClick={() => onOpen(chat.id)}
      sx={{
        width: '100%',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'stretch',
        gap: 0.5,
        px: 1.75,
        py: 1.125,
        textAlign: 'left',
      }}
    >
      <Box sx={{ display: 'flex', alignItems: 'baseline', gap: 1, pr: 4 }}>
        <Typography noWrap sx={{ flex: 1, fontSize: 14, fontWeight: 500 }}>
          {chat.name || `Chat ${chat.id.replace('chat_', '').split('_')[0]}`}
        </Typography>
        <Typography sx={{ fontSize: 11.5, color: 'text.secondary', flexShrink: 0 }}>
          {historyTimeLabel(chat.lastUpdated)}
        </Typography>
      </Box>
      {chat.preview && (
        <Typography noWrap sx={{ fontSize: 12.5, color: chatColors.neutral[800] }}>
          {chat.preview}
        </Typography>
      )}
      <Box sx={{ display: 'flex', gap: 0.75, flexWrap: 'wrap' }}>
        <StatusTag label={chat.engine === 'eve' ? `eve · ${chat.agent}` : chat.agent} />
        {chat.subagentCount > 0 && <SubagentCountTag count={chat.subagentCount} />}
      </Box>
    </ButtonBase>
    <Tooltip title="Delete chat">
      <IconButton
        className="delete"
        size="small"
        aria-label={`Delete ${chat.name || 'chat'}`}
        onClick={() => onDelete(chat.id)}
        sx={{ position: 'absolute', right: 8, bottom: 6, opacity: 0, transition: 'opacity 120ms' }}
      >
        <DeleteOutlineIcon sx={{ fontSize: 16 }} />
      </IconButton>
    </Tooltip>
  </Box>
);

/**
 * The history view: replaces the thread and the composer. Top-level chats only (a subagent is
 * opened from its parent), searchable, grouped by day.
 */
const ChatHistoryView = ({
  chats,
  query,
  onQueryChange,
  activeChatId,
  loading,
  onOpen,
  onDelete,
}) => {
  const groups = groupHistory(chats, query);

  return (
    <Box sx={{ flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
      <Box sx={{ px: 1.75, py: 1.5, borderBottom: '1px solid', borderColor: 'divider' }}>
        <TextField
          fullWidth
          size="small"
          value={query}
          onChange={(event) => onQueryChange(event.target.value)}
          placeholder="Search chats"
          slotProps={{
            htmlInput: { 'aria-label': 'Search chats' },
            input: {
              sx: { fontSize: 14 },
              startAdornment: (
                <InputAdornment position="start">
                  <SearchIcon sx={{ fontSize: 16 }} />
                </InputAdornment>
              ),
            },
          }}
        />
      </Box>

      <Box sx={{ flex: 1, overflowY: 'auto', pb: 1.5 }}>
        {groups.length === 0 && (
          <Typography sx={{ px: 1.75, py: 3, fontSize: 13, color: 'text.secondary' }}>
            {loading
              ? 'Loading chats…'
              : query.trim()
                ? 'No chats match that search.'
                : 'No chats yet.'}
          </Typography>
        )}
        {groups.map((group) => (
          <Box key={group.label}>
            <Typography sx={{ ...kickerSx, px: 1.75, pt: 1.5, pb: 0.5 }}>{group.label}</Typography>
            {group.chats.map((chat) => (
              <HistoryItem
                key={chat.id}
                chat={chat}
                active={chat.id === activeChatId}
                onOpen={onOpen}
                onDelete={onDelete}
              />
            ))}
          </Box>
        ))}
      </Box>
    </Box>
  );
};

export default ChatHistoryView;
