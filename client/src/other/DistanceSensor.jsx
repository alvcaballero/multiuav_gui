import React, { useId } from 'react'; // useId para IDs únicos
import PropTypes from 'prop-types';

/**
 * Componente React para visualizar un sensor de distancia triangular.
 * La punta del triángulo representa la parte más cercana al sensor.
 * Una parte del triángulo se rellena con color (verde, amarillo, rojo)
 * según el valor de la distancia recibida.
 * Utiliza SVG con una máscara para controlar el área de relleno.
 *
 * @param {number} distance - El valor de la distancia (entero entre 1 y 10).
 * @param {number} [width=150] - Ancho del SVG.
 * @param {number} [height=150] - Alto del SVG.
 * @param {Array} [limits=[0,10]] - Valores max - min del sensor . 
*/
const DistanceSensor = ({ distance, width = 100, height = 100, limits = [0, 10] }) => { // Default size adjusted for easier positioning
  const uniqueId = useId(); // Genera un ID único y estable para la máscara
  const maskId = `distanceMask-${uniqueId}`;

  // Aseguramos que la distancia esté dentro del rango 1-10
  const clampedDistance = Math.max(limits[0], Math.min(limits[1], distance));

  // Calculamos el nivel de relleno vertical (0-100) basado en la distancia (1-10)
  // Una distancia de 10 llena completamente (fillY = 100)
  // Una distancia de 1 llena una pequeña parte desde la punta (fillY = 10)
  const fillY = clampedDistance * height / (limits[1] - limits[0]);
  const limitcolor1 = limits[0] + 0.75 * (limits[1] - limits[0])
  const limitcolor2 = limits[0] + 0.3 * (limits[1] - limits[0])

  // Determinamos el color de relleno basado en rangos de distancia
  let fillColor;
  if (clampedDistance >= limits[0] && clampedDistance <= limitcolor2) {
    fillColor = 'red'; // Distancia baja, seguro
  } else if (clampedDistance >= limitcolor2 && clampedDistance <= limitcolor1) {
    fillColor = 'yellow'; // Distancia media, precaución
  } else { // clampedDistance >= 8 && clampedDistance <= 10
    fillColor = 'green'; // Distancia alta, cerca
  }

  // Definimos los puntos del polígono que actuará como máscara.
  const maskPoints = `0,${height} 0,${fillY} ${width},${fillY} ${width},${height}`;
  const PolygonPoints = `${width / 2},0 0,${height} ${width},${height}`

  return (
    <svg width={width} height={height} >
      <defs>
        {/* Definimos la máscara SVG. Las áreas blancas en la máscara revelan el contenido */}
        {/* Usamos el ID único */}
        <mask id={maskId}>
          {/* El polígono blanco define el área que será visible del elemento masked */}
          <polygon points={maskPoints} fill="white" />
        </mask>
      </defs>

      {/* El polígono base del triángulo (el contorno) */}
      <polygon
        points={PolygonPoints}
        stroke="none" // Color del contorno
        fill="lightgray"
        strokeWidth="5" // Grosor del contorno aumentado un poco
      />

      {/* El polígono coloreado que será enmascarado */}
      {/* Es un triángulo completo que cubre toda el área */}
      {/* La máscara con ID único determinará qué parte de este polígono es visible */}
      <polygon
        points={PolygonPoints}
        fill={fillColor} // El color determinado por la distancia
        mask={`url(#${maskId})`} // Aplicamos la máscara usando el ID único
      />
    </svg>
  );
};

DistanceSensor.propTypes = {
  distance: PropTypes.number.isRequired,
  width: PropTypes.number,
  height: PropTypes.number,
};

export default DistanceSensor;