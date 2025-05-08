import React from 'react';
import PropTypes from 'prop-types';
import DistanceSensor from './DistanceSensor'; // Ajusta la ruta si es necesario
import './DroneSensorVisualizer.css'; // Importa el archivo CSS para estilos
import { mapIconKey, mapIcons, frontIcons } from '../Mapview/preloadImages';


/**
 * Componente que visualiza los sensores de un dron en vistas superior y frontal.
 * Muestra un dron simple y utiliza el componente DistanceSensor para graficar
 * las lecturas de distancia en diferentes direcciones.
 */
const DroneSensorVisualizer = ({ sensorData }) => {
  // Tamaños para los sensores visuales
  const sensorSize = 100; // Ancho y alto de los componentes DistanceSensor
  const droneSizeTop = 80; // Tamaño del cuadrado del dron en vista superior
  const droneSizeFrontWidth = 120; // Ancho del rectángulo del dron en vista frontal
  const droneSizeFrontHeight = 50; // Alto del rectángulo del dron en vista frontal

  return (
    <div className="drone-visualizer-container">
      {/* Vista Superior */}
      <div className="view-container top-view">
        <div className="visualization-area">
          {/* Dron en vista superior (cuadrado simple) */}
          <img  src={mapIcons[mapIconKey('ArrowMap')]} alt='' style={{width:droneSizeTop}}/>

          {/* Sensores de Vista Superior */}
          {/* Frontal (apunta hacia arriba, 0deg rotación) */}
          <div
            className="sensor-pos front"
            style={{
              width: sensorSize,
              height: sensorSize,
              top: `-${sensorSize * 0.2}px`, // Ajuste de posición
              left: '50%',
              transform: 'translate(-50%, 0) rotate(180deg)',
            }}
          >
          <div className="sensor-pos">Up: {sensorData.up} m</div>
            <DistanceSensor distance={sensorData.front} width={sensorSize} height={sensorSize} />
          </div>

          {/* Trasero (apunta hacia abajo, 180deg rotación) */}
          <div
            className="sensor-pos back"
            style={{
              width: sensorSize,
              height: sensorSize,
              bottom: `-${sensorSize * 0.2}px`, // Ajuste de posición
              left: '50%',
              transform: 'translate(-50%, 0) ',
            }}
          >
          <div className="sensor-pos">Up: {sensorData.up} m</div>
            <DistanceSensor distance={sensorData.back} width={sensorSize} height={sensorSize} />
          </div>

          {/* Izquierdo (apunta hacia la izquierda, -90deg rotación) */}
          <div
            className="sensor-pos left"
            style={{
              width: sensorSize,
              height: sensorSize,
              top: '50%',
              left: `-${sensorSize * 0.2}px`, // Ajuste de posición
              transform: 'translate(0, -50%) rotate(90deg)',
            }}
          >
            <DistanceSensor distance={sensorData.left} width={sensorSize} height={sensorSize} />
          </div>

          {/* Derecho (apunta hacia la derecha, 90deg rotación) */}
          <div
            className="sensor-pos right"
            style={{
              width: sensorSize,
              height: sensorSize,
              top: '50%',
              right: `-${sensorSize * 0.2}px`, // Ajuste de posición
              transform: 'translate(0, -50%) rotate(-90deg)',
            }}
          >
            <DistanceSensor distance={sensorData.right} width={sensorSize} height={sensorSize} />
          </div>

        </div>
        {/* Valores numéricos de sensores de Vista Superior */}
        <div className="sensor-values">
          <p>Front: {sensorData.front} m</p>
          <p>Back: {sensorData.back} m</p>
          <p>Left: {sensorData.left} m</p>
          <p>Right: {sensorData.right} m</p>
        </div>
      </div>

      {/* Vista Frontal */}
      <div className="view-container front-view">
        <div className="visualization-area">
           {/* Dron en vista frontal (rectángulo simple) */}
          <svg className="drone-shape front" width={droneSizeFrontWidth} height={droneSizeFrontHeight} viewBox={`0 0 ${droneSizeFrontWidth} ${droneSizeFrontHeight}`}>
             <rect x="0" y="0" width={droneSizeFrontWidth} height={droneSizeFrontHeight} fill="#555" rx="5" ry="5" />
          </svg>

          {/* Sensores de Vista Frontal */}
          {/* Superior (apunta hacia arriba, 0deg rotación) */}
          <div
            className="sensor-pos up"
             style={{
              width: sensorSize,
              height: sensorSize,
              top: `-${sensorSize * 0.2}px`, // Ajuste de posición
              left: '50%',
              transform: 'translate(-50%, 0)',
            }}
          >
            <DistanceSensor distance={sensorData.up} width={sensorSize} height={sensorSize} />
          </div>

          {/* Inferior (apunta hacia abajo, 180deg rotación) */}
          <div
            className="sensor-pos down"
            style={{
              width: sensorSize,
              height: sensorSize,
              bottom: `-${sensorSize * 0.2}px`, // Ajuste de posición
              left: '50%',
              transform: 'translate(-50%, 0) rotate(180deg)',
            }}
          >
            <DistanceSensor distance={sensorData.down} width={sensorSize} height={sensorSize} />
          </div>
          <div className="sensor-pos">Up: {sensorData.up} m</div>

        </div>
         {/* Valores numéricos de sensores de Vista Frontal */}
        <div className="sensor-values">
          <p>Up: {sensorData.up} m</p>
          <p>Down: {sensorData.down} m</p>
        </div>
      </div>
    </div>
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