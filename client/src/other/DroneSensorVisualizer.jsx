import React, { useState } from 'react';
import PropTypes from 'prop-types';
import DistanceSensor from './DistanceSensor'; // Ajusta la ruta si es necesario
import { mapIconKey, mapIcons, frontIcons } from '../Mapview/preloadImages';
import LinearGauge from './LinearGauge';



const getSensorStyle = (direction, sensorWidth, sensorHeight, containerSize, droneSize) => {

  const center = containerSize / 2;
  const halfDrone = droneSize / 2;
  const halfConeSize = sensorWidth / 2;

  let style = {
    position: "absolute",
    zIndex: 1,
    transform: 'translate(-50%, 0)',
  }
  const textLabelStyle = {
    position: 'absolute',
    fontSize: '1.2em',
    color: 'black',
    fontWeight: 'bold',
    zIndex: 2, // Encima del cono
  };

  switch (direction) {
    case 'front':
      style.top = 0
      style.left = center
      style.transform = 'translate(-50%, 0) rotate(180deg)'
      textLabelStyle.top = style.top;
      textLabelStyle.left = style.left - 20;
      break;

    case 'back':
      style.bottom = 0
      style.left = center
      textLabelStyle.bottom = style.bottom;
      textLabelStyle.left = style.left - 20;
      break;

    case 'left':
      style.top = center
      style.transform = 'translate(0, -50%) rotateZ(180deg)'
      style.left = 0

      textLabelStyle.top = style.top - 10;
      textLabelStyle.left = style.left + 40;
      break;

    case 'right':
      style.top = center
      style.right = 0
      style.transform = 'translate(0, -50%)'

      textLabelStyle.top = style.top - 10;
      textLabelStyle.right = style.right + 40;
      break;
    default:
      break;
  }


  return { coneStyle: style, textStyle: textLabelStyle };

}



/**
 * Componente que visualiza los sensores de un dron en vistas superior y frontal.
 * Muestra un dron simple y utiliza el componente DistanceSensor para graficar
 * las lecturas de distancia en diferentes direcciones.
 */
const DroneSensorVisualizer = ({
  sensorData = {
    down: 10,
    front: 10,
    left: 10,
    back: 10,
    right: 10,
    up: 10,
  },
  sensorConfig = {
    down: [0, 15],
    front: [0, 15],
    left: [0, 12],
    back: [0, 12],
    right: [0, 12],
    up: [0, 20],
  }
}) => {
  const [currentAltitude, setCurrentAltitude] = useState(10.0);
  const [currentS1, setCurrents1] = useState(10.0);
  const [currentS2, setCurrents2] = useState(10.0);


  // Función para manejar el cambio del valor con un slider de ejemplo
  const handleSliderChange = (event) => {
    setCurrentAltitude(parseFloat(event.target.value));
  };
  // Tamaños para los sensores visuales
  const sensorSize = 150; // Ancho y alto de los componentes DistanceSensor
  const droneSizeTop = 100; // Tamaño del cuadrado del dron en vista superior
  const CONTAINER_SIZE = 350; // Tamaño del contenedor en píxeles
  const sensorWidth = sensorSize * 1.5
  const sensorHeight = sensorSize


  //GET STYLES FOR SENSOR 
  const { coneStyle: styleFrontCone, textStyle: styleFrontText } = getSensorStyle("front", sensorWidth, sensorHeight, CONTAINER_SIZE, droneSizeTop)
  const { coneStyle: styleBackCone, textStyle: styleBackText } = getSensorStyle("back", sensorWidth, sensorHeight, CONTAINER_SIZE, droneSizeTop)
  const { coneStyle: styleLeftCone, textStyle: styleLeftText } = getSensorStyle("left", sensorWidth, sensorHeight, CONTAINER_SIZE, droneSizeTop)
  const { coneStyle: styleRightCone, textStyle: styleRightText } = getSensorStyle("right", sensorWidth, sensorHeight, CONTAINER_SIZE, droneSizeTop)
  //margin: '20px auto', // Centrar el contenedor
  //overflow: 'hidden', // Ocultar cualquier cosa posicionada fuera
  //borderRadius: '8px', // Bordes redondeados

  //position: 'relative',
  //
  return (
    <>
      <div style={{
        display: "flex",
        width: CONTAINER_SIZE + 240,
        height: CONTAINER_SIZE,
        border: '1px solid #ccc',
        backgroundColor: '#f9f9f9', // Fondo muy claro
        boxShadow: '2px 2px 8px rgba(0,0,0,0.1)', // Sombra suave
        flexWrap: "wrap",
        justifyContent: "space-around"
      }}>

        <div style={{
          width: CONTAINER_SIZE,
          height: CONTAINER_SIZE,
          position: 'relative',
          justifyContent: 'center', /* Centra el dron */
          alignItems: 'center'
        }}>
          {/* Representacion del drone */}
          <img src={mapIcons[mapIconKey('ArrowMap')]} alt=''
            style={{
              width: droneSizeTop,
              position: "absolute",
              top: "50%",
              left: "50%",
              transform: "translate(-50%,-50%)",
              zIndex: 2
            }} />

          <div style={styleFrontText}>{sensorData.front.toFixed(0)}m</div>
          <div style={styleFrontCone}>
            <DistanceSensor distance={sensorData.front} limits={sensorConfig.front} width={sensorWidth} height={sensorHeight} />
          </div>

          <div style={styleBackText}>{sensorData.back.toFixed(0)}m</div>
          <div style={styleBackCone} >
            <DistanceSensor distance={sensorData.back} limits={sensorConfig.back} width={sensorWidth} height={sensorHeight} />
          </div>

          <div style={styleLeftText}>{sensorData.left.toFixed(0)}m</div>
          <div style={styleLeftCone} >
            <DistanceSensor distance={sensorData.left} limits={sensorConfig.left}  width={sensorHeight} height={sensorWidth} orientation={"h"}  />
          </div>

          <div style={styleRightText}>{sensorData.right.toFixed(0)}m</div>
          <div style={styleRightCone} >
            <DistanceSensor distance={sensorData.right} limits={sensorConfig.right} width={sensorHeight} height={sensorWidth} orientation={"h"} />
          </div>

        </div>
        <div style={{
          width: CONTAINER_SIZE,
          height: CONTAINER_SIZE,
          position: 'relative',
          display: "flex",
          flexDirection: "column",
          flex: 1,
          justifyContent: 'center', /* Centra el dron */
          alignItems: 'center'
        }}>
          <LinearGauge
            value={currentAltitude}
            sensorValue={[sensorData.up, sensorData.down]}
            sensorLimits={{up:sensorConfig.up,down:sensorConfig.down}}
            height={CONTAINER_SIZE}
          />
        </div>
      </div >
    </>
  );
};

// Definición de PropTypes para la estructura de sensorData
DroneSensorVisualizer.propTypes = {
  sensorData: PropTypes.shape({
    front: PropTypes.number.isRequired,
    back: PropTypes.number.isRequired,
    left: PropTypes.number.isRequired,
    right: PropTypes.number.isRequired,
    up: PropTypes.number.isRequired,
    down: PropTypes.number.isRequired,
  }).isRequired,
};

export default DroneSensorVisualizer;