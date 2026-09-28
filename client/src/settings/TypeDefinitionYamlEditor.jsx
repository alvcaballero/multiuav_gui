import React, { useEffect, useState } from 'react';
import { Box, Button, Typography } from '@mui/material';
import Editor from 'react-simple-code-editor';
import Prism from 'prismjs';
import 'prismjs/components/prism-yaml';
import 'prismjs/themes/prism.css';
import { useCatch } from '../reactHelper';

const highlightYaml = (code) => Prism.highlight(code, Prism.languages.yaml, 'yaml');

/**
 * Raw view/edit of an ElementType's stored semantic/parametric model file
 * (e.g. a wtsem-format .type.yaml) — an asset like icon/model3d, not a
 * structured form for its (much richer) schema. Its `state_defaults` back
 * `parameterDefs` when the type's own DB copy is empty — see
 * elementTypesModel.resolveEffectiveParameterDefs on the server.
 */
const TypeDefinitionYamlEditor = ({ typeId, isNew, onContentChange }) => {
  const [content, setContentState] = useState('');
  const [status, setStatus] = useState('idle'); // idle | loading | saving | ok | error

  // Mirrors every change up to the parent (used by TypeYamlCompareViewer to
  // "Comprobar" whatever is currently in the editor, saved or not) without
  // turning this into a controlled component — this stays the source of
  // truth for its own content, the parent just gets a copy.
  const setContent = (value) => {
    setContentState(value);
    onContentChange?.(value);
  };

  useEffect(() => {
    if (!typeId || isNew) return undefined;
    const controller = new AbortController();
    setStatus('loading');
    fetch(`/api/markers/types/${typeId}/definition`, { signal: controller.signal })
      .then((response) => (response.ok ? response.text() : ''))
      .then((text) => setContent(text))
      .catch(() => {})
      .finally(() => setStatus('idle'));
    return () => controller.abort();
    // setContent is a stable local wrapper, not a prop-derived value —
    // including it would just re-fetch on every parent render for no reason.
    // eslint-disable-next-line @eslint-react/exhaustive-deps
  }, [typeId, isNew]);

  const handleSave = useCatch(async () => {
    setStatus('saving');
    const body = new FormData();
    body.append('file', new Blob([content], { type: 'text/yaml' }), 'definition.yaml');
    const response = await fetch(`/api/markers/types/${typeId}/definition`, {
      method: 'POST',
      body,
    });
    if (!response.ok) {
      setStatus('error');
      throw new Error(await response.text());
    }
    setStatus('ok');
    setTimeout(() => setStatus('idle'), 2000);
  });

  const handleLoadFromDisk = (file) => {
    const reader = new FileReader();
    reader.onload = () => setContent(String(reader.result));
    reader.readAsText(file);
  };

  if (isNew) {
    return (
      <Typography variant="body2" color="text.secondary">
        Guardá el tipo primero para poder subir su definición YAML.
      </Typography>
    );
  }

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      <Typography variant="body2" color="text.secondary">
        Modelo semántico/paramétrico del tipo (formato wtsem-type). Sus <code>state_defaults</code>{' '}
        se usan como parámetros por defecto cuando este tipo todavía no tiene{' '}
        <code>parameterDefs</code> propios.
      </Typography>

      <Button component="label" variant="outlined" sx={{ alignSelf: 'flex-start' }}>
        Cargar desde archivo
        <input
          hidden
          type="file"
          accept=".yaml,.yml"
          onChange={(event) => event.target.files[0] && handleLoadFromDisk(event.target.files[0])}
        />
      </Button>

      <Box
        sx={{
          border: '1px solid',
          borderColor: 'divider',
          borderRadius: 1,
          height: 400,
          overflow: 'auto',
          '& textarea:focus': { outline: 'none' },
        }}
      >
        <Editor
          value={content}
          onValueChange={setContent}
          highlight={highlightYaml}
          padding={10}
          style={{ fontFamily: '"Roboto Mono", monospace', fontSize: 13, minHeight: '100%' }}
        />
      </Box>

      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
        <Button variant="contained" onClick={handleSave} disabled={status === 'saving' || !content}>
          Guardar definición
        </Button>
        {status === 'ok' && (
          <Typography variant="body2" color="success.main">
            Guardado
          </Typography>
        )}
        {status === 'error' && (
          <Typography variant="body2" color="error.main">
            YAML inválido — no se guardó
          </Typography>
        )}
      </Box>
    </Box>
  );
};

export default TypeDefinitionYamlEditor;
