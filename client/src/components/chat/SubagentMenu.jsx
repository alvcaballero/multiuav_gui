import { Box, ButtonBase, Link, Popover, Typography } from '@mui/material';
import { alpha } from '@mui/material/styles';
import SmartToyOutlinedIcon from '@mui/icons-material/SmartToyOutlined';
import StatusTag from './StatusTag';
import { kickerSx } from './chatTheme';
import { formatTimestamp } from '../../common/util/chatMessage';

// The colour says how it went: the accent while it runs, teal once it finished, red when it failed.
const STATUS_LOOK = {
  running: {
    tone: 'running',
    label: 'Running',
    line: 'primary.main',
    strip: 'primary.100',
  },
  error: { tone: 'failed', label: 'Failed', line: 'error.main', strip: 'error.tint' },
  done: {
    tone: 'done',
    label: 'Completed',
    line: 'subagent.main',
    strip: 'subagent.strip',
  },
};

const capitalize = (text) => (text ? text.charAt(0).toUpperCase() + text.slice(1) : text);

/** "Planner" for the agent type `planner`: what a subagent is called in the menu and its title. */
export const subagentName = (subagent) => capitalize(subagent.agentType) || 'Subagent';

const rowSx = (active) => ({
  display: 'flex',
  alignItems: 'center',
  gap: 1.25,
  p: 1,
  textAlign: 'left',
  justifyContent: 'flex-start',
  bgcolor: active ? 'primary.100' : 'transparent',
  '&:hover': { bgcolor: (theme) => alpha(theme.palette.primary.main, 0.1) },
  borderRadius: '2px',
});

const approvalsText = (count) =>
  count === 1 ? '1 approval waiting' : `${count} approvals waiting`;

/**
 * The dropdown that replaces the tab strip: the main thread, and the subagents it spawned, newest
 * first. Opening one is navigation to that chat — a subagent is a chat of its own.
 */
const SubagentMenu = ({
  anchorEl,
  onClose,
  subagents,
  mainMeta,
  activeChatId,
  rootChatId,
  onSelect,
}) => (
  <Popover
    open={Boolean(anchorEl)}
    anchorEl={anchorEl}
    onClose={onClose}
    anchorOrigin={{ vertical: 'bottom', horizontal: 'right' }}
    transformOrigin={{ vertical: 'top', horizontal: 'right' }}
    slotProps={{
      paper: { sx: { width: 320, maxWidth: 'calc(100vw - 16px)', p: 0.75, mt: 0.5 } },
    }}
  >
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: '2px' }}>
      <Typography sx={{ ...kickerSx, px: 1, py: 0.5 }}>This chat</Typography>
      <ButtonBase onClick={() => onSelect(rootChatId)} sx={rowSx(activeChatId === rootChatId)}>
        <SmartToyOutlinedIcon sx={{ fontSize: 16 }} />
        <Box sx={{ display: 'flex', flexDirection: 'column', flex: 1, minWidth: 0 }}>
          <Typography sx={{ fontSize: 13.5, fontWeight: 500 }}>Main thread</Typography>
          <Typography noWrap sx={{ fontSize: 12, color: 'text.secondary' }}>
            {mainMeta}
          </Typography>
        </Box>
      </ButtonBase>

      <Typography
        sx={{
          ...kickerSx,
          px: 1,
          pt: 1,
          pb: 0.5,
          mt: 0.5,
          borderTop: '1px solid',
          borderColor: 'divider',
        }}
      >
        Subagents spawned
      </Typography>
      {subagents.map((subagent) => {
        const tag = STATUS_LOOK[subagent.status] ?? STATUS_LOOK.done;
        return (
          <ButtonBase
            key={subagent.chatId}
            onClick={() => onSelect(subagent.chatId)}
            sx={{
              ...rowSx(activeChatId === subagent.chatId),
              ml: '15px',
              borderLeft: 3,
              borderLeftColor: tag.line,
            }}
          >
            <Box sx={{ display: 'flex', flexDirection: 'column', flex: 1, minWidth: 0 }}>
              <Typography sx={{ fontSize: 13.5, fontWeight: 500 }}>
                {subagentName(subagent)}
              </Typography>
              <Typography noWrap sx={{ fontSize: 12, color: 'text.secondary' }}>
                {[
                  formatTimestamp(subagent.createdAt),
                  subagent.approvalsWaiting > 0 ? approvalsText(subagent.approvalsWaiting) : null,
                ]
                  .filter(Boolean)
                  .join(' · ')}
              </Typography>
            </Box>
            <StatusTag tone={tag.tone} label={tag.label} />
          </ButtonBase>
        );
      })}
    </Box>
  </Popover>
);

/** Shown under the title bar while inside a subagent: who spawned it, and the way back. */
export const SubagentStrip = ({ subagent, meta, onGoToMain }) => {
  const tag = STATUS_LOOK[subagent?.status] ?? STATUS_LOOK.done;
  return (
    <Box
      sx={{
        display: 'flex',
        alignItems: 'center',
        gap: 1,
        px: 1.75,
        py: 1,
        bgcolor: tag.strip,
        borderBottom: '1px solid',
        borderColor: 'divider',
        fontSize: 12,
        color: 'neutral.800',
        flexShrink: 0,
      }}
    >
      <Box component="span" sx={{ flex: 1, minWidth: 0 }}>
        Spawned by{' '}
        <Link
          component="button"
          type="button"
          onClick={onGoToMain}
          sx={{ fontSize: 'inherit', verticalAlign: 'baseline' }}
        >
          Main
        </Link>
        {meta ? ` · ${meta}` : ''}
      </Box>
      <StatusTag tone={tag.tone} label={tag.label} />
    </Box>
  );
};

export default SubagentMenu;
