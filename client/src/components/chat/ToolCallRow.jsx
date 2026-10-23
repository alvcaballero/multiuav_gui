import { memo, useState } from 'react';
import { Box, Button, ButtonBase, CircularProgress, Collapse, Typography } from '@mui/material';
import { alpha } from '@mui/material/styles';
import BuildOutlinedIcon from '@mui/icons-material/BuildOutlined';
import CheckIcon from '@mui/icons-material/Check';
import ErrorOutlineIcon from '@mui/icons-material/ErrorOutlineOutlined';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import { MONO_FONT, OFFSCREEN_SKIP } from './chatTheme';
import { PayloadBlock, FormattedPayload } from './CodeBlock';
import {
  durationLabel,
  getCachedResultSummary,
  getMessageView,
  getPrettyJson,
  summarizeArgs,
} from '../../common/util/chatMessage';
import { displayToolName } from '../../common/util/inputRequest';

// A collapsed tool row is the 7px + 7px padding around one 18px line, plus its border.
const collapsedRowSx = { ...OFFSCREEN_SKIP, containIntrinsicSize: 'auto 34px' };

/** The "Show mission" shortcut a mission-producing call carries. */
const showMission = async (args, name) => {
  const missionPlanId = args?.missionPlanId || args?.missionPlanid;
  const isValidateMission = name === 'validate_mission' && args?.mission;
  try {
    if (missionPlanId) {
      await fetch(`/api/missions/plans/show/${missionPlanId}`, { method: 'GET' });
    }

    if (isValidateMission) {
      await fetch('/api/missions/showXYZ', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          version: args.mission.version || '3',
          name: args.mission.name || 'Validated Mission',
          route: args.mission.route,
          global_origin: args.mission.global_origin || {
            lat: 41.687222,
            lng: -8.8477450788,
            alt: 0,
          },
        }),
      });
    }
  } catch (error) {
    console.error('Error creating mission:', error);
  }
};

const StatusIcon = ({ status }) => {
  if (status === 'running') return <CircularProgress size={14} thickness={4} />;
  if (status === 'error') return <ErrorOutlineIcon sx={{ fontSize: 14 }} color="error" />;
  if (status === 'done') return <CheckIcon sx={{ fontSize: 14, color: 'success.main' }} />;
  return null;
};

/**
 * A tool call and its result, as one collapsible row (the handoff's "tool call row").
 *
 * `running` is the caller's knowledge, not the row's: a call with no result is in flight only if
 * the turn is still open, and otherwise it never got an answer — nothing to spin about.
 */
const ToolCallRow = ({ call, output, running = false }) => {
  const [open, setOpen] = useState(false);

  const callView = call ? getMessageView(call) : null;
  const outputView = output ? getMessageView(output) : null;

  const name = displayToolName(callView?.name ?? outputView?.name) ?? 'tool';
  const args = callView?.content;
  const result = outputView?.content;
  const hasError = outputView ? getCachedResultSummary(result).hasError : false;
  const status = hasError ? 'error' : outputView ? 'done' : running ? 'running' : null;
  const duration = call && output ? durationLabel(call.timestamp, output.timestamp) : null;

  const imageSrc = result?.image_data
    ? `data:${result.mime_type || 'image/jpeg'};base64,${result.image_data}`
    : null;
  const canShowMission =
    Boolean(args?.missionPlanId || args?.missionPlanid) ||
    (name === 'validate_mission' && Boolean(args?.mission));

  return (
    <Box
      sx={{
        border: '1px solid',
        borderStyle: status === 'running' ? 'dashed' : 'solid',
        borderColor: status === 'running' ? 'primary.400' : 'divider',
        borderRadius: 1,
        overflow: 'hidden',
        bgcolor: status === 'running' ? 'primary.100' : 'background.paper',
        flexShrink: 0,
        ...collapsedRowSx,
      }}
    >
      <Box sx={{ display: 'flex', alignItems: 'center' }}>
        <ButtonBase
          onClick={() => setOpen((value) => !value)}
          aria-expanded={open}
          sx={{
            flex: 1,
            minWidth: 0,
            display: 'flex',
            alignItems: 'center',
            gap: 1,
            px: 1.25,
            py: 0.875,
            textAlign: 'left',
            justifyContent: 'flex-start',
            '&:hover': { bgcolor: (theme) => alpha(theme.palette.text.primary, 0.05) },
          }}
        >
          <BuildOutlinedIcon sx={{ fontSize: 14, color: 'primary.700' }} />
          <Typography sx={{ fontFamily: MONO_FONT, fontSize: 12.5, flexShrink: 0 }}>
            {name}
          </Typography>
          <Typography noWrap sx={{ fontSize: 12, color: 'text.secondary', flex: 1, minWidth: 0 }}>
            {summarizeArgs(args)}
          </Typography>
          {duration && (
            <Typography sx={{ fontSize: 11, color: 'text.secondary', flexShrink: 0 }}>
              {duration}
            </Typography>
          )}
          <StatusIcon status={status} />
          <ExpandMoreIcon
            sx={{
              fontSize: 14,
              transition: 'transform 120ms',
              transform: open ? 'rotate(180deg)' : 'none',
            }}
          />
        </ButtonBase>
        {canShowMission && (
          <Button
            size="small"
            onClick={() => showMission(args, name)}
            sx={{ mr: 1, flexShrink: 0 }}
          >
            Show mission
          </Button>
        )}
      </Box>

      <Collapse in={open} unmountOnExit>
        <Box
          sx={{
            borderTop: '1px solid',
            borderColor: 'divider',
            px: 1.25,
            pt: 1,
            pb: 1.25,
            display: 'grid',
            gap: 1,
          }}
        >
          {call && <PayloadBlock label="Input" value={getPrettyJson(args)} />}
          {imageSrc && (
            <Box
              component="img"
              src={imageSrc}
              alt={`${name} result`}
              sx={{ maxWidth: '100%', maxHeight: 320, display: 'block' }}
            />
          )}
          {output && <FormattedPayload label="Output" content={result} />}
          {!output && (
            <Typography sx={{ fontSize: 12, color: 'text.secondary' }}>
              {running ? 'Waiting for the result…' : 'No result was recorded for this call.'}
            </Typography>
          )}
        </Box>
      </Collapse>
    </Box>
  );
};

export default memo(ToolCallRow);
