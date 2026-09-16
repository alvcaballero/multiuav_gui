import React, { useState } from 'react';
import { useParams } from 'react-router-dom';
import {
  TextField,
  Button,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  Avatar,
} from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import { useCatch } from '../reactHelper';
import SettingsMenu from './components/SettingsMenu';
import useSettingsStyles from './common/useSettingsStyles';
import EditItemView from './components/EditItemView';
import GeometryFields, { DEFAULT_CIRCLE_GEOMETRY } from '../shared/components/GeometryFields';
import { invalidateMarkerTypesCache } from '../hooks/useMarkerTypes';

const SettingsElementTypesPageEdit = () => {
  const { classes } = useSettingsStyles();
  const { id } = useParams();
  const isNew = !id;

  const [item, setItem] = useState(null);

  const validate = () => item && item.id && item.name;

  const handleUploadIcon = useCatch(async (file) => {
    const body = new FormData();
    body.append('file', file);
    const response = await fetch(`/api/markers/types/${item.id}/icon`, { method: 'POST', body });
    if (!response.ok) throw Error(await response.text());
    setItem(await response.json());
    invalidateMarkerTypesCache();
  });

  const handleUploadModel = useCatch(async (file) => {
    const body = new FormData();
    body.append('file', file);
    const response = await fetch(`/api/markers/types/${item.id}/model`, { method: 'POST', body });
    if (!response.ok) throw Error(await response.text());
    setItem(await response.json());
    invalidateMarkerTypesCache();
  });

  return (
    <EditItemView
      endpoint="markers/types"
      item={item}
      setItem={setItem}
      defaultItem={{
        id: '',
        name: '',
        description: '',
        color: '#1976d2',
        attributes: { geometry: DEFAULT_CIRCLE_GEOMETRY },
      }}
      validate={validate}
      onItemSaved={() => invalidateMarkerTypesCache()}
      menu={<SettingsMenu />}
      breadcrumbs={['settingsTitle', 'Element Types']}
    >
      {item && (
        <>
          <Accordion defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Datos generales</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <TextField
                required
                label="Id"
                value={item.id}
                disabled={!isNew}
                helperText={isNew ? 'Identificador único, no editable luego de creado' : undefined}
                onChange={(event) => setItem({ ...item, id: event.target.value.trim() })}
              />
              <TextField
                required
                label="Name"
                value={item.name}
                onChange={(event) => setItem({ ...item, name: event.target.value })}
              />
              <TextField
                label="Description"
                value={item.description || ''}
                onChange={(event) => setItem({ ...item, description: event.target.value })}
              />
              <TextField
                label="Color"
                type="color"
                value={item.color || '#1976d2'}
                onChange={(event) => setItem({ ...item, color: event.target.value })}
              />
            </AccordionDetails>
          </Accordion>

          <Accordion defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Geometría por defecto</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <GeometryFields
                value={item.attributes?.geometry}
                onChange={(geometry) =>
                  setItem({ ...item, attributes: { ...item.attributes, geometry } })
                }
              />
            </AccordionDetails>
          </Accordion>

          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Ícono y modelo 3D</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              {isNew ? (
                <Typography variant="body2" color="text.secondary">
                  Guardá el tipo primero para poder subir ícono y modelo 3D.
                </Typography>
              ) : (
                <>
                  {item.icon && (
                    <Avatar variant="rounded" src={item.icon} sx={{ width: 48, height: 48 }} />
                  )}
                  <Button component="label" variant="outlined">
                    Subir ícono
                    <input
                      hidden
                      type="file"
                      accept="image/png,image/svg+xml,image/jpeg"
                      onChange={(event) =>
                        event.target.files[0] && handleUploadIcon(event.target.files[0])
                      }
                    />
                  </Button>
                  <Button component="label" variant="outlined">
                    Subir modelo 3D (glb/gltf)
                    <input
                      hidden
                      type="file"
                      accept=".glb,.gltf"
                      onChange={(event) =>
                        event.target.files[0] && handleUploadModel(event.target.files[0])
                      }
                    />
                  </Button>
                  {item.model3d && (
                    <Typography variant="body2" color="text.secondary">
                      {item.model3d}
                    </Typography>
                  )}
                </>
              )}
            </AccordionDetails>
          </Accordion>
        </>
      )}
    </EditItemView>
  );
};

export default SettingsElementTypesPageEdit;
