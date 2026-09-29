import { createTheme } from '@mui/material';
import { loadImage, prepareIcon } from './mapUtil';
import { routeColor } from '../../shared/routeColors';

import { grey } from '@mui/material/colors';
import backgroundSvg from '../../resources/images/background.svg';
import directionSvg from '../../resources/images/direction.svg';
import backgroundBorderSvg from '../../resources/images/background_border.svg';
import backgroundDirectionSvg from '../../resources/images/background_direction.svg';

import planeSvg from '../../resources/images/icon/plane.svg';
import helicopterSvg from '../../resources/images/icon/helicopter.svg';
import droneSvg from '../../resources/images/icon/drone1.svg';
import birdSvg from '../../resources/images/icon/bird.svg';
import dronedjiSvg from '../../resources/images/icon/drone2.svg';
import dronePx4Svg from '../../resources/images/icon/drone3.svg';
import triangleSvg from '../../resources/images/icon/triangle.svg';
import locationPointSvg from '../../resources/images/icon/locationPoint.svg';
import powerTowerSvg from '../../resources/images/icon/PowerTower1.svg';
import windTurbineSvg from '../../resources/images/icon/windTurbine.svg';
import solarPanelSvg from '../../resources/images/icon/SolarPanel.svg';
import RectangleSvg from '../../resources/images/icon/Rectangle.svg';
import ArrowMapSvg from '../../resources/images/icon/ArrowMap.svg';
import ArrowMapFronSvg from '../../resources/images/icon/ArrowMap2.svg';

import FrontDroneSvg from '../../resources/images/icon/drone-svgrepo.svg';

export const mapIcons = {
  helicopter: helicopterSvg,
  plane: planeSvg,
  drone: droneSvg,
  dji_M210: dronedjiSvg,
  dji_M300: dronedjiSvg,
  dji_M600: FrontDroneSvg,
  px4: planeSvg,
  catec: dronePx4Svg,
  fuvex: planeSvg,
  griffin: birdSvg,
  ArrowMap: ArrowMapSvg,
  default: planeSvg,
};

export const frontIcons = {
  helicopter: FrontDroneSvg,
  plane: FrontDroneSvg,
  drone: FrontDroneSvg,
  dji_M210: FrontDroneSvg,
  dji_M300: FrontDroneSvg,
  dji_M600: FrontDroneSvg,
  px4: FrontDroneSvg,
  catec: FrontDroneSvg,
  fuvex: FrontDroneSvg,
  ArrowMap: ArrowMapFronSvg,
  default: FrontDroneSvg,
};

export const mapIconKey = (category) => {
  switch (category) {
    case 'dji_M210_noetic':
    case 'dji_M210_melodic_rtk':
    case 'dji_M210_melodic':
    case 'dji_M210_noetic_rtk':
      return 'dji_M210';
    case 'dji_M300':
    case 'dji_M300_rtk':
      return 'dji_M300';
    default:
      return mapIcons.hasOwnProperty(category) ? category : 'default';
  }
};

export const mapImages = {};

let imagesReadyResolve;
export const imagesReady = new Promise((resolve) => {
  imagesReadyResolve = resolve;
});

// Targets (inspection elements) have their own image namespace, separate from the
// devices' `{category}-{color}`: a target layer asks for `target-<type>`, and the
// map's missing-image resolver (mapInstance.js) resolves it with resolveTargetImage.
// A type that can't be resolved falls back to TARGET_DEFAULT_IMAGE — a missing
// device image, instead, still surfaces as an error.
export const TARGET_IMAGE_PREFIX = 'target-';
export const TARGET_DEFAULT_IMAGE = `${TARGET_IMAGE_PREFIX}default`;

export const targetImageId = (type) =>
  type == null || type === '' ? TARGET_DEFAULT_IMAGE : `${TARGET_IMAGE_PREFIX}${type}`;

// `type` is either a fixed image name (e.g. 'locPoint') or an ElementType id, whose
// icon is fetched the first time a layer asks for it (so a type created after
// startup gets its icon without a reload). The result is cached under `imageId`,
// the default included, so a type without an icon isn't refetched on every render.
export const resolveTargetImage = async (imageId) => {
  await imagesReady;
  const type = imageId.slice(TARGET_IMAGE_PREFIX.length);
  if (mapImages[type]) {
    mapImages[imageId] = mapImages[type];
  } else if (/^\d+$/.test(type)) {
    try {
      mapImages[imageId] = await prepareIcon(await loadImage(`/api/markers/types/${type}/icon`));
    } catch {
      console.warn(`No icon for element type "${type}", using the default target icon`);
      mapImages[imageId] = mapImages[TARGET_DEFAULT_IMAGE];
    }
  } else {
    console.warn(`Unknown target type "${type}", using the default target icon`);
    mapImages[imageId] = mapImages[TARGET_DEFAULT_IMAGE];
  }
  return mapImages[imageId];
};

// Route images are `<shape>-<routeKey>`, tinted with routeColor(routeKey): the
// waypoint (`background`), the oriented waypoint (`backgroundDirection`) and the
// ring around a device flying that route (`mission`). They are generated the first
// time a layer asks for them, so there is no limit on the number of routes.
const ROUTE_IMAGE_ID = /^(background|backgroundDirection|mission)-(\d+)$/;
const routeImageShapes = {};

export const resolveRouteImage = async (imageId) => {
  const match = ROUTE_IMAGE_ID.exec(imageId);
  if (!match) return undefined;
  await imagesReady;
  const [, shape, routeKey] = match;
  mapImages[imageId] = prepareIcon(routeImageShapes[shape], null, routeColor(routeKey));
  return mapImages[imageId];
};

const theme = createTheme({
  palette: {
    neutral: { main: grey[500] },
  },
});

export default async () => {
  const background = await loadImage(backgroundSvg);
  const backgroundBorder = await loadImage(backgroundBorderSvg);
  const backgroundDirection = await loadImage(backgroundDirectionSvg);
  routeImageShapes.background = background;
  routeImageShapes.backgroundDirection = backgroundDirection;
  routeImageShapes.mission = backgroundBorder;

  mapImages.background = await prepareIcon(background);
  mapImages.direction = await prepareIcon(await loadImage(directionSvg));

  mapImages.base = await prepareIcon(await loadImage(RectangleSvg));
  mapImages.item = await prepareIcon(await loadImage(triangleSvg));
  mapImages[TARGET_DEFAULT_IMAGE] = mapImages.item;
  mapImages.powerTower = await prepareIcon(await loadImage(powerTowerSvg));
  mapImages.windTurbine = await prepareIcon(await loadImage(windTurbineSvg));
  mapImages.solarPanel = await prepareIcon(await loadImage(solarPanelSvg));
  mapImages.locPoint = await prepareIcon(await loadImage(locationPointSvg));

  await Promise.all(
    Object.keys(mapIcons).map(async (category) => {
      let icon;
      try {
        icon = await loadImage(mapIcons[category]);
      } catch (e) {
        console.warn(`preloadImages: failed to load icon for "${category}"`, e);
        return;
      }
      ['info', 'success', 'error', 'neutral'].forEach((color) => {
        mapImages[`${category}-${color}`] = prepareIcon(
          background,
          icon,
          theme.palette[color].main,
        );
      });
    }),
  );
  console.log('preload icon');
  console.log(mapImages);
  imagesReadyResolve();
};
