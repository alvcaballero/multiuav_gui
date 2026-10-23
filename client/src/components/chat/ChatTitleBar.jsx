import { Box, ButtonBase, IconButton, Tooltip, Typography } from '@mui/material';
import AccountTreeOutlinedIcon from '@mui/icons-material/AccountTreeOutlined';
import AddIcon from '@mui/icons-material/Add';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import AutoAwesomeOutlinedIcon from '@mui/icons-material/AutoAwesomeOutlined';
import CloseIcon from '@mui/icons-material/Close';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import HistoryIcon from '@mui/icons-material/History';
import { HEADING_FONT } from './chatTheme';

const iconButtonSx = { width: 30, height: 30, '&:hover': { bgcolor: 'titleBar.hover' } };
const iconSx = { fontSize: 18 };

/**
 * The title bar of the panel: where you are (the assistant, a subagent, the history), the
 * subagent switcher, and the actions. There are two ways to start a chat — one per engine — and
 * each is its own button, so which engine answers is chosen by the click and never by a setting.
 */
const ChatTitleBar = ({
  title,
  subtitle,
  showBack,
  subagentCount,
  subagentRunning,
  subagentMenuOpen,
  eveEnabled,
  onBack,
  onToggleSubagents,
  onHistory,
  onNewLegacyChat,
  onNewEveChat,
  onClose,
}) => (
  <Box
    sx={{
      display: 'flex',
      alignItems: 'center',
      gap: 1,
      py: 1,
      pr: 1,
      pl: 1.75,
      bgcolor: 'titleBar.main',
      borderBottom: 1,
      borderBottomColor: 'titleBar.border',
      flexShrink: 0,
    }}
  >
    {showBack && (
      <Tooltip title="Back">
        <IconButton aria-label="Back" onClick={onBack} sx={{ ...iconButtonSx, ml: -0.75 }}>
          <ArrowBackIcon sx={iconSx} />
        </IconButton>
      </Tooltip>
    )}

    <Box sx={{ display: 'flex', flexDirection: 'column', flex: 1, minWidth: 0 }}>
      <Typography
        noWrap
        sx={{
          fontFamily: HEADING_FONT,
          fontWeight: 700,
          fontSize: 17,
          letterSpacing: '0.3em',
          textTransform: 'uppercase',
          lineHeight: 1.25,
        }}
      >
        {title}
      </Typography>
      <Typography noWrap sx={{ fontSize: 12, color: 'neutral.700', lineHeight: 1.3 }}>
        {subtitle}
      </Typography>
    </Box>

    <Box sx={{ display: 'flex', gap: '2px', alignItems: 'center' }}>
      {subagentCount > 0 && (
        <Tooltip title="Subagent chats">
          <ButtonBase
            onClick={onToggleSubagents}
            aria-haspopup="true"
            aria-expanded={subagentMenuOpen}
            aria-label={`Subagent chats (${subagentCount})`}
            sx={{
              display: 'flex',
              alignItems: 'center',
              gap: 0.625,
              height: 30,
              px: 1,
              mr: 0.5,
              border: 1,
              borderColor: 'titleBar.control',
              borderRadius: '3px',
              fontSize: 12.5,
              '&:hover': { bgcolor: 'titleBar.hover' },
            }}
          >
            <AccountTreeOutlinedIcon sx={{ fontSize: 15 }} />
            <Box component="span" sx={{ fontWeight: 500 }}>
              {subagentCount}
            </Box>
            {subagentRunning && (
              <Box
                component="span"
                title="Running"
                sx={{ width: 6, height: 6, bgcolor: 'primary.main', borderRadius: '50%' }}
              />
            )}
            <ExpandMoreIcon sx={{ fontSize: 14 }} />
          </ButtonBase>
        </Tooltip>
      )}
      <Tooltip title="Chat history">
        <IconButton aria-label="Chat history" onClick={onHistory} sx={iconButtonSx}>
          <HistoryIcon sx={iconSx} />
        </IconButton>
      </Tooltip>
      <Tooltip title="New chat · legacy">
        <IconButton aria-label="New legacy chat" onClick={onNewLegacyChat} sx={iconButtonSx}>
          <AddIcon sx={iconSx} />
        </IconButton>
      </Tooltip>
      {eveEnabled && (
        <Tooltip title="New chat · eve">
          <IconButton aria-label="New eve chat" onClick={onNewEveChat} sx={iconButtonSx}>
            <AutoAwesomeOutlinedIcon sx={iconSx} />
          </IconButton>
        </Tooltip>
      )}
      {onClose && (
        <Tooltip title="Close">
          <IconButton aria-label="Close" onClick={onClose} sx={iconButtonSx}>
            <CloseIcon sx={iconSx} />
          </IconButton>
        </Tooltip>
      )}
    </Box>
  </Box>
);

export default ChatTitleBar;
