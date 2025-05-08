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
 */
const DistanceSensor = ({ distance, width = 100, height = 100 }) => { // Default size adjusted for easier positioning
  const uniqueId = useId(); // Genera un ID único y estable para la máscara
  const maskId = `distanceMask-${uniqueId}`;

  // Aseguramos que la distancia esté dentro del rango 1-10
  const clampedDistance = Math.max(1, Math.min(10, distance));

  // Calculamos el nivel de relleno vertical (0-100) basado en la distancia (1-10)
  // Una distancia de 10 llena completamente (fillY = 100)
  // Una distancia de 1 llena una pequeña parte desde la punta (fillY = 10)
  const fillY = clampedDistance * 10;

  // Determinamos el color de relleno basado en rangos de distancia
  let fillColor;
  if (clampedDistance >= 1 && clampedDistance <= 4) {
    fillColor = 'red'; // Distancia baja, seguro
  } else if (clampedDistance >= 5 && clampedDistance <= 7) {
    fillColor = 'yellow'; // Distancia media, precaución
  } else { // clampedDistance >= 8 && clampedDistance <= 10
    fillColor = 'green'; // Distancia alta, cerca
  }

  // Definimos los puntos del polígono que actuará como máscara.
  // Este polígono es un triángulo que va desde la punta (50,0) hasta una base
  // en la coordenada Y calculada (fillY), con ancho proporcional.
  // Los puntos son: Punta(50,0), Esquina Inferior Izquierda((50-fillY/2), fillY), Esquina Inferior Derecha((50+fillY/2), fillY)
  //const maskPoints = `50,0 ${50 - fillY / 2},${fillY} ${50 + fillY / 2},${fillY}`;
  const maskPoints = `0,100 ${50 - fillY / 2},${fillY} ${50 + fillY / 2},${fillY} 100,100`;


  return (
    <svg width={width} height={height} viewBox="0 0 100 100">
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
        points="50,0 0,100 100,100"
        stroke="none" // Color del contorno
        fill="lightgray"
        strokeWidth="5" // Grosor del contorno aumentado un poco
      />

      {/* El polígono coloreado que será enmascarado */}
      {/* Es un triángulo completo que cubre toda el área */}
      {/* La máscara con ID único determinará qué parte de este polígono es visible */}
      <polygon
        points="50,0 0,100 100,100"
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