import React from 'react';
import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Box,
  Divider,
  Paper,
  Tab,
  Typography,
} from '@mui/material';
import { TabContext, TabList, TabPanel } from '@mui/lab';
import ExpandMore from '@mui/icons-material/ExpandMore';

import MapView from '../../map/core/MapView';
import { MapMissions } from '../../map/mission/MapMissions';
import MapMarkers from '../../map/environment/MapMarkers';
import RoutesList from '../../components/mission/RoutesList';
import SelectField from '../../shared/components/SelectField';
import SelectList from '../../components/ui/SelectList';
import BaseSettings from '../../components/planning/BaseSettings';

const noop = () => null;

const MissionMapPanel = ({
  classes,
  missions,
  routePath,
  missionMarkers,
  dataMission,
  dataParam,
  tabValue,
  onTabChange,
}) => (
  <div>
    <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
      Mapa de mission
    </Typography>
    <div style={{ width: '100%', height: '500px', position: 'relative' }}>
      <MapView>
        <MapMissions filtereddeviceid={-1} routes={routePath} />
        <MapMarkers markers={missionMarkers} />
      </MapView>
      <Paper square elevation={3} className={classes.missionMapOverlay}>
        <TabContext value={tabValue}>
          <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
            <TabList onChange={onTabChange} aria-label="lab API tabs example">
              <Tab label="mission" value="1" />
              <Tab label="Planning" value="2" />
            </TabList>
          </Box>
          <TabPanel value="1">
            {routePath && (
              <RoutesList
                mission={missions.mission}
                setmission={noop}
                setScrool={noop}
                NoEdit={true}
              />
            )}
          </TabPanel>
          <TabPanel value="2">
            {missions.task && (
              <>
                <SelectField
                  emptyValue={null}
                  fullWidth
                  label="objetive"
                  value={missions.task.case}
                  endpoint="/api/planning/missionstype"
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                />
                <Accordion>
                  <AccordionSummary expandIcon={<ExpandMore />}>
                    <Typography>Interest elements</Typography>
                  </AccordionSummary>
                  <AccordionDetails className={classes.details}>
                    {missions.task.locations && <SelectList Data={missions.task.locations} />}
                  </AccordionDetails>
                </Accordion>
                <Divider />
                <Accordion>
                  <AccordionSummary expandIcon={<ExpandMore />}>
                    <Typography>Devices</Typography>
                  </AccordionSummary>
                  <AccordionDetails className={classes.details}>
                    {dataMission && dataParam && (
                      <BaseSettings
                        data={dataMission}
                        param={dataParam}
                        setData={noop}
                        goToBase={noop}
                      />
                    )}
                  </AccordionDetails>
                </Accordion>
              </>
            )}
          </TabPanel>
        </TabContext>
      </Paper>
    </div>
  </div>
);

export default MissionMapPanel;
