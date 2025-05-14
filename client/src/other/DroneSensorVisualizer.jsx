import React, { useState } from 'react';
import PropTypes from 'prop-types';
import DistanceSensor from './DistanceSensor'; // Ajusta la ruta si es necesario
import { mapIconKey, mapIcons, frontIcons } from '../Mapview/preloadImages';
import LinearGauge from './LinearGauge';



const getSensorStyle = (direction, sensorSize, containerSize, droneSize) => {

  const sensorWidth = sensorSize
  const sensorheight = sensorSize
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
      style.top = sensorSize * 0.2
      style.left = center
      style.transform = 'translate(-50%, 0) rotate(180deg)'
      textLabelStyle.top = style.top;
      textLabelStyle.left = style.left - 20;
      break;

    case 'back':
      style.bottom = sensorSize * 0.2
      style.left = center
      textLabelStyle.bottom = style.bottom;
      textLabelStyle.left = style.left - 20;
      break;

    case 'left':
      style.top = center
      style.left = sensorSize * 0.1
      style.transform = 'translate(0, -50%) rotate(90deg)'

      textLabelStyle.top = style.top - 10;
      textLabelStyle.left = style.left;
      break;

    case 'right':
      style.top = center
      style.right = sensorSize * 0.1
      style.transform = 'translate(0, -50%) rotate(-90deg)'

      textLabelStyle.top = style.top - 10;
      textLabelStyle.right = style.right;
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
const DroneSensorVisualizer = ({ sensorData }) => {
  const [currentAltitude, setCurrentAltitude] = useState(10.0);
  const [currentS1, setCurrents1] = useState(10.0);
  const [currentS2, setCurrents2] = useState(10.0);


  // Función para manejar el cambio del valor con un slider de ejemplo
  const handleSliderChange = (event) => {
    setCurrentAltitude(parseFloat(event.target.value));
  };
  // Tamaños para los sensores visuales
  const sensorSize = 120; // Ancho y alto de los componentes DistanceSensor
  const droneSizeTop = 80; // Tamaño del cuadrado del dron en vista superior
  const CONTAINER_SIZE = 350; // Tamaño del contenedor en píxeles
  const sensorWidth = sensorSize
  const sensorHeight = sensorSize * 1.5


  //GET STYLES FOR SENSOR 
  const { coneStyle: styleFrontCone, textStyle: styleFrontText } = getSensorStyle("front", sensorSize, CONTAINER_SIZE, droneSizeTop)
  const { coneStyle: styleBackCone, textStyle: styleBackText } = getSensorStyle("back", sensorSize, CONTAINER_SIZE, droneSizeTop)
  const { coneStyle: styleLeftCone, textStyle: styleLeftText } = getSensorStyle("left", sensorSize, CONTAINER_SIZE, droneSizeTop)
  const { coneStyle: styleRightCone, textStyle: styleRightText } = getSensorStyle("right", sensorSize, CONTAINER_SIZE, droneSizeTop)
  //margin: '20px auto', // Centrar el contenedor
  //overflow: 'hidden', // Ocultar cualquier cosa posicionada fuera
  //borderRadius: '8px', // Bordes redondeados

  //position: 'relative',
  //
  return (
    <>
      <div style={{
        display: "flex",
        width: CONTAINER_SIZE * 2.5,
        height: CONTAINER_SIZE * 1,
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

          <div style={styleFrontText}>{sensorData.front.toFixed(1)}m</div>
          <div style={styleFrontCone}>
            <DistanceSensor distance={sensorData.front} width={sensorHeight} height={sensorSize} />
          </div>

          <div style={styleBackText}>{sensorData.back.toFixed(1)}m</div>
          <div style={styleBackCone} >
            <DistanceSensor distance={sensorData.back} width={sensorHeight} height={sensorSize} />
          </div>

          <div style={styleLeftText}>{sensorData.left.toFixed(1)}m</div>
          <div style={styleLeftCone} >
            <DistanceSensor distance={sensorData.left} width={sensorHeight} height={sensorSize} style={styleLeftCone} />
          </div>

          <div style={styleRightText}>{sensorData.right.toFixed(1)}m</div>
          <div style={styleRightCone} >
            <DistanceSensor distance={sensorData.right} width={sensorHeight} height={sensorSize} />
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
            sensorValue={[currentS1, currentS2]}
          />
        </div>

      </div >

      <input
        type="range"
        min="0"
        max="10"
        step="0.1"
        value={currentAltitude}
        onChange={handleSliderChange}
        style={{ width: '300px', marginTop: '20px' }}
      />
      <input
        type="range"
        min="0"
        max="10"
        step="0.1"
        value={currentS1}
        onChange={(event) => { setCurrents1(parseFloat(event.target.value)) }}

        style={{ width: '300px', marginTop: '20px' }}
      />
      <input
        type="range"
        min="0"
        max="10"
        step="0.1"
        value={currentS2}
        onChange={(event) => { setCurrents2(parseFloat(event.target.value)) }}
        style={{ width: '300px', marginTop: '20px' }}
      />
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