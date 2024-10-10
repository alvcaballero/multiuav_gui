import palette from '../common/palette';
import { loadImage, prepareIcon } from './mapUtil';

import backgroundSvg from '../resources/images/background.svg';
import directionSvg from '../resources/images/direction.svg';
import backgroundBorderSvg from '../resources/images/background_border.svg';
import backgroundDirectionSvg from '../resources/images/background_direction.svg';

import planeSvg from '../resources/images/icon/plane.svg';
import helicopterSvg from '../resources/images/icon/helicopter.svg';
import droneSvg from '../resources/images/icon/drone1.svg';
import birdSvg from '../resources/images/icon/bird.svg';
import dronedjiSvg from '../resources/images/icon/drone2.svg';
import dronePx4Svg from '../resources/images/icon/drone3.svg';
import triangleSvg from '../resources/images/icon/triangle.svg';
import powerTowerSvg from '../resources/images/icon/PowerTower1.svg';
import windTurbineSvg from '../resources/images/icon/WindTurbine.svg';
import solarPanelSvg from '../resources/images/icon/wind_turbine.svg';
import RectangleSvg from '../resources/images/icon/Rectangle.svg';
import gruaPng from '../resources/lastimages/bridge_crane_small.png';

import FrontDroneSvg from '../resources/images/icon/drone-svgrepo.svg';

export const colors = {
  0: '#F34C28',
  1: '#F39A28',
  2: '#1EC910',
  3: '#1012C9',
  4: '#C310C9',
  5: '#1FDBF1',
  6: '#F6FD04',
  7: '#808080',
};

export const mapIcons = {
  helicopter: helicopterSvg,
  plane: planeSvg,
  drone: droneSvg,
  dji_M210_noetic: dronedjiSvg,
  dji_M210_melodic: dronedjiSvg,
  dji_M210_noetic_rtk: dronedjiSvg,
  dji_M210_melodic_rtk: dronedjiSvg,
  dji_M300: dronedjiSvg,
  dji_M300_rtk: dronedjiSvg,
  dji_M600: FrontDroneSvg,
  px4: planeSvg,
  catec: dronePx4Svg,
  fuvex: planeSvg,
  griffin: birdSvg,
  default: planeSvg,
};

export const frontIcons = {
  helicopter: FrontDroneSvg,
  plane: FrontDroneSvg,
  drone: FrontDroneSvg,
  dji_M210_noetic: FrontDroneSvg,
  dji_M210_melodic: FrontDroneSvg,
  dji_M210_noetic_rtk: FrontDroneSvg,
  dji_M210_melodic_rtk: FrontDroneSvg,
  dji_M300: FrontDroneSvg,
  dji_M300_rtk: dronedjiSvg,
  dji_M600: FrontDroneSvg,
  px4: FrontDroneSvg,
  catec: FrontDroneSvg,
  fuvex: FrontDroneSvg,
  default: FrontDroneSvg,
};

export const mapIconKey = (category) => (mapIcons.hasOwnProperty(category) ? category : 'default');

export const mapImages = {};

export default async () => {
  const background = await loadImage(backgroundSvg);
  const backgroundBorder = await loadImage(backgroundBorderSvg);
  const backgroundDirection = await loadImage(backgroundDirectionSvg);

  mapImages.background = await prepareIcon(background);
  mapImages.item = await prepareIcon(await loadImage(triangleSvg));
  mapImages.powerTower = await prepareIcon(await loadImage(powerTowerSvg));
  mapImages.windTurbine = await prepareIcon(await loadImage(windTurbineSvg));
  mapImages.solarPanel = await prepareIcon(await loadImage(solarPanelSvg));
  mapImages.grua = await prepareIcon(await loadImage(gruaPng));


  mapImages.base = await prepareIcon(await loadImage(RectangleSvg));
  mapImages.direction = await prepareIcon(await loadImage(directionSvg));
  Object.keys(palette.colors_devices).forEach(async (color) => {
    mapImages[`background-${color}`] = await prepareIcon(background, null, colors[color]);
    mapImages[`mission-${color}`] = await prepareIcon(backgroundBorder, null, colors[color]);
    mapImages[`backgroundDirection-${color}`] = await prepareIcon(backgroundDirection, null, colors[color]);
  });
  await Promise.all(
    Object.keys(mapIcons).map(async (category) => {
      const results = [];
      ['primary', 'positive', 'negative', 'neutral'].forEach((color) => {
        results.push(
          loadImage(mapIcons[category]).then((icon) => {
            mapImages[`${category}-${color}`] = prepareIcon(background, icon, palette.colors[color]);
          })
        );
      });
      await Promise.all(results);
    })
  );
  //console.log('preload icon');
  //console.log(mapImages);
};
