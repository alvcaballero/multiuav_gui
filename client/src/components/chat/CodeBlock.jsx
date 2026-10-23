import { useState } from 'react';
import { Box, Button, IconButton, Tooltip, Typography, useTheme } from '@mui/material';
import ContentCopyOutlinedIcon from '@mui/icons-material/ContentCopyOutlined';
import CheckIcon from '@mui/icons-material/Check';
import { PrismLight as SyntaxHighlighter } from 'react-syntax-highlighter';
import json from 'react-syntax-highlighter/dist/esm/languages/prism/json';
import yaml from 'react-syntax-highlighter/dist/esm/languages/prism/yaml';
import bash from 'react-syntax-highlighter/dist/esm/languages/prism/bash';
import python from 'react-syntax-highlighter/dist/esm/languages/prism/python';
import javascript from 'react-syntax-highlighter/dist/esm/languages/prism/javascript';
import markdown from 'react-syntax-highlighter/dist/esm/languages/prism/markdown';
import prism from 'react-syntax-highlighter/dist/esm/styles/prism/prism';
import { kickerSx, MONO_FONT } from './chatTheme';
import { getFormattedResult } from '../../common/util/chatMessage';

// PrismLight ships no grammars: each one is opted into explicitly. The full `Prism` build pulls
// refractor's ~594 languages into the bundle to render what is, in practice, JSON and a little
// shell. Register only what a transcript can realistically contain — an unregistered language
// degrades to plain text, it does not throw.
SyntaxHighlighter.registerLanguage('json', json);
SyntaxHighlighter.registerLanguage('yaml', yaml);
SyntaxHighlighter.registerLanguage('bash', bash);
SyntaxHighlighter.registerLanguage('python', python);
SyntaxHighlighter.registerLanguage('javascript', javascript);
SyntaxHighlighter.registerLanguage('markdown', markdown);

/**
 * Characters drawn before a payload is cut off behind an explicit "show all" click.
 *
 * Tool results in this codebase's own transcripts average ~2.8 KB and peak near 146 KB — thousands
 * of lines of JSON that no one reads in a <pre>; they get skimmed, then copied. Drawing all of it
 * is a DOM node per token for the highlighted kind and a very tall box for the plain one, and an
 * expanded row pays it again on every open. Capping it bounds the worst case by design instead of
 * by luck, while the copy button still puts the FULL payload on the clipboard.
 */
const MAX_SHOWN_CHARS = 2000;

/** Cuts at the last line break inside the budget, so the preview ends on a whole line. */
const truncateToLine = (text) => {
  const slice = text.slice(0, MAX_SHOWN_CHARS);
  const lastBreak = slice.lastIndexOf('\n');
  return lastBreak > 0 ? slice.slice(0, lastBreak) : slice;
};

const useTruncated = (value) => {
  const [showFull, setShowFull] = useState(false);
  const text = typeof value === 'string' ? value : String(value ?? '');
  const isTruncated = !showFull && text.length > MAX_SHOWN_CHARS;
  return {
    text,
    shown: isTruncated ? truncateToLine(text) : text,
    isTruncated,
    showAll: () => setShowFull(true),
  };
};

const useCopy = (text) => {
  const [copied, setCopied] = useState(false);
  const copy = () => {
    // Always the full payload, never what happens to be on screen.
    navigator.clipboard.writeText(text);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };
  return { copied, copy };
};

const CopyButton = ({ copied, onCopy }) => (
  <Tooltip title={copied ? 'Copied' : 'Copy all'}>
    <IconButton onClick={onCopy} size="small" aria-label="Copy" sx={{ p: 0.5 }}>
      {copied ? (
        <CheckIcon sx={{ fontSize: 14 }} />
      ) : (
        <ContentCopyOutlinedIcon sx={{ fontSize: 14 }} />
      )}
    </IconButton>
  </Tooltip>
);

const TruncationNote = ({ shown, total, onShowAll }) => (
  <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, px: 1, py: 0.5 }}>
    <Typography sx={{ ...kickerSx, letterSpacing: '0.06em', fontFamily: MONO_FONT, flex: 1 }}>
      {`${shown.toLocaleString()} of ${total.toLocaleString()} characters`}
    </Typography>
    <Button size="small" onClick={onShowAll} sx={{ minHeight: 0, py: 0 }}>
      Show all
    </Button>
  </Box>
);

const preSx = {
  m: 0,
  p: 1,
  bgcolor: 'background.surface',
  fontFamily: MONO_FONT,
  fontSize: 12,
  lineHeight: 1.45,
  whiteSpace: 'pre-wrap',
  overflowWrap: 'anywhere',
};

/**
 * A labelled `<pre>` of a tool's input or output — INPUT / OUTPUT in the handoff. Plain on
 * purpose: highlighting every tool payload is a cost with nothing to show for it.
 */
export const PayloadBlock = ({ label, value }) => {
  const { text, shown, isTruncated, showAll } = useTruncated(value);
  const { copied, copy } = useCopy(text);

  return (
    <Box sx={{ display: 'grid', gap: 0.75 }}>
      <Box sx={{ display: 'flex', alignItems: 'center' }}>
        <Typography sx={{ ...kickerSx, flex: 1, letterSpacing: '0.1em' }}>{label}</Typography>
        <CopyButton copied={copied} onCopy={copy} />
      </Box>
      <Box>
        <Box component="pre" sx={preSx}>
          {shown}
        </Box>
        {isTruncated && (
          <Box sx={{ bgcolor: 'background.surface' }}>
            <TruncationNote shown={shown.length} total={text.length} onShowAll={showAll} />
          </Box>
        )}
      </Box>
    </Box>
  );
};

/** A tool payload of any shape, formatted the way the transcript always has (MCP unwrapping). */
export const FormattedPayload = ({ label, content }) => {
  const { text } = getFormattedResult(content);
  return <PayloadBlock label={label} value={text} />;
};

/** A fenced block inside the assistant's markdown: highlighted, with a copy button. */
export const CodeBlock = ({ language, value }) => {
  const theme = useTheme();
  const { text, shown, isTruncated, showAll } = useTruncated(value);
  const { copied, copy } = useCopy(text);

  return (
    <Box sx={{ position: 'relative', my: 1, border: 1, borderColor: 'divider' }}>
      <Box
        sx={{
          display: 'flex',
          alignItems: 'center',
          px: 1,
          bgcolor: 'grey.200',
          borderBottom: 1,
          borderBottomColor: 'divider',
        }}
      >
        <Typography sx={{ ...kickerSx, flex: 1, fontFamily: MONO_FONT, letterSpacing: 0 }}>
          {language || 'text'}
        </Typography>
        <CopyButton copied={copied} onCopy={copy} />
      </Box>
      <SyntaxHighlighter
        language={language || 'text'}
        style={prism}
        customStyle={{
          margin: 0,
          borderRadius: 0,
          fontSize: 12,
          // customStyle is an inline style, not sx: it takes the colour, not its palette path.
          background: theme.palette.background.surface,
          fontFamily: MONO_FONT,
        }}
      >
        {shown}
      </SyntaxHighlighter>
      {isTruncated && (
        <TruncationNote shown={shown.length} total={text.length} onShowAll={showAll} />
      )}
    </Box>
  );
};
