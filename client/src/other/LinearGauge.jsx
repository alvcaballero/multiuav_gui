import React, { useState, useEffect } from 'react';
import './LinearGauge.css'; // Importamos los estilos
import { mapIconKey, mapIcons, frontIcons } from '../Mapview/preloadImages';


const LinearGauge = ({
    value,                 // Valor actual a mostrar (ej: 10.0)
    sensorValue = [7, 5],
    zeroValue = 0,         // Valor que representa la barra azul (ej: 0)
    range = 50,            // Rango total de valores visibles en el medidor (ej: 50 para -15 a 35 si 'value' es 10)
    height = 300,          // Altura del medidor en píxeles
    width = 230,            // Ancho del cuerpo del medidor en píxeles
    tickInterval = 5,      // Intervalo entre las marcas pequeñas (ej: 5 unidades)
    majorTickInterval = 25, // Intervalo entre las marcas grandes con etiquetas (ej: 25 unidades)
    unit = 'ALT (m)',      // Unidad a mostrar junto al valor (ej: "ALT (m)")
    indicatorColor = 'rgb(51, 51, 51)', // Color del indicador/flecha
    zeroBarColor = 'skyblue',    // Color de la barra azul del cero
    tickColor = '#ccc',         // Color de las marcas (ticks)
    labelColor = 'black',      // Color de las etiquetas de los números
    backgroundColor = '#f8f8f8', // Color de fondo del medidor
}) => {
    // Calculamos el rango visible del medidor, centrado alrededor del valor actual
    const [ticks, setTicks] = useState([])
    const [labels, setLabels] = useState([])
    const [ColorSensorUp, setColorSensorUp] = useState("gray")
    const [ColorSensorDown, setColorSensorDown] = useState("gray")
    const [sensorUp, setSensorUp] = useState(0)
    const [sensorDown, setSensorDown] = useState(0)

    const currentMin = value - range / 2;
    const currentMax = value + range / 2;

    // Función para mapear un valor a una posición vertical en píxeles
    // El medidor va de abajo (valores más pequeños) a arriba (valores más grandes)
    const getValuePosition = (val) => {
        // Normaliza el valor a una escala de 0 a 1 dentro del rango actual
        const normalizedValue = (val - currentMin) / (range);
        // Convierte a posición en píxeles (0 es abajo, height es arriba)
        return height * (1 - normalizedValue);
    };

    const zeroPos = getValuePosition(zeroValue);
    const indicatorPos = getValuePosition(value);
    const zeroPosX = 30;
    const halfdrone = 30;

    // Generamos las marcas (ticks) y etiquetas

    // Determinamos el inicio y fin para la generación de ticks,
    // asegurando que cubran el rango visible y sean múltiplos del intervalo.
    const startValueForTicks = Math.floor(currentMin / tickInterval) * tickInterval;
    const endValueForTicks = Math.ceil(currentMax / tickInterval) * tickInterval;

    const setFillColor = (clampedDistance) => {
        let fillColor = 'gray'; // Distancia baja, seguro

        if (clampedDistance >= 1 && clampedDistance <= 4) {
            fillColor = 'red'; // Distancia baja, seguro
        } else if (clampedDistance >= 5 && clampedDistance <= 7) {
            fillColor = 'yellow'; // Distancia media, precaución
        } else { // clampedDistance >= 8 && clampedDistance <= 10
            fillColor = 'lightgray'; // Distancia alta, cerca
        }
        return fillColor
    }

    useEffect(() => {
        const myticks = [];
        const mylabels = [];



        for (let i = startValueForTicks; i <= endValueForTicks; i += tickInterval) {

            let mystyle = {
                position: "absolute",
                left: 0, /* Alinea los ticks a la derecha del track del medidor */
                height: "2px",
                transform: "translateY(-50%)"
            }

            const pos = getValuePosition(i);
            if (pos >= -10 && pos <= height + 10) { // Renderiza solo si está cerca o dentro de la vista
                const isMajorTick = Math.abs(i % majorTickInterval) < tickInterval / 2; // Comprueba si es una marca principal (con tolerancia)
                mystyle.width = isMajorTick ? '15px' : '10px'
                mystyle.top = pos
                mystyle.backgroundColor = tickColor
                myticks.push(<div key={`tick-${i}`} style={mystyle} />);

                if (isMajorTick) {
                    let labelStyle = {
                        position: "absolute",
                        left: 30, /* Posición a la derecha de los ticks */
                        fontSize: "18px",
                        whiteSpace: "nowrap",
                        top: pos - 10,
                        color: labelColor
                    }
                    mylabels.push(<div key={`label-${i}`} style={labelStyle}>{i}</div>);
                }
            }
        }
        setTicks(myticks)
        setLabels(mylabels)
    }, [value])

    useEffect(() => {
        let s1 = Math.max(1, Math.min(10, sensorValue[0]));

        setSensorUp(s1 * (height / 2) / 10)
        setColorSensorUp(setFillColor(s1))

        let s2 = Math.max(1, Math.min(10, sensorValue[1]));
        setSensorDown(s2 * (height / 2) / 10)
        setColorSensorDown(setFillColor(s2))
    }, [sensorValue])



    return (
        <div
            style={{
                position: 'relative',
                width: `${width}px`,
                height: `${height}px`,
                backgroundColor: backgroundColor,
                overflow: "hidden"
            }}
        >
            <div style={{
                position: "absolute",
                height: "100%",
                width: "50px",
                left: zeroPosX + halfdrone,
                borderLeft: `4px solid ${tickColor}`,
                borderTop: `4px solid ${tickColor}`,
                borderBottom: `4px solid ${tickColor}`,
                borderColor: tickColor,
            }}>

            </div>
            {/* Barra del Cero */}
            <div
                style={{
                    position: "absolute",
                    left: zeroPosX,
                    top: "50%",
                    transform: "translateY(-50%)",
                    zIndex: 3
                }} >
                <img src={frontIcons[mapIconKey('ArrowMap')]} alt='' style={{ width: halfdrone * 2 }} />
            </div>


            {/* Contenedor de Ticks y Labels */}
            <div
                style={{
                    position: "absolute",
                    left: halfdrone + zeroPosX,
                    top: 0,
                    width: "100%",
                    height: "100%",
                }}
            >
                <div className="gauge-ticks-container">
                    {ticks}
                </div>
                <div className="gauge-labels-container">
                    {labels}
                </div>
            </div>
            {/* sensor s1 */}
            <div style={{
                bottom: "50%",
                position: "absolute",
                left: zeroPosX,
                width: 30,
                height: sensorUp, // Grosor de la barra azul
            }} >

                <div style={{
                    position: "absolute",
                    bottom: 0,
                    width: 20,
                    left: 10,
                    height: "100%", // Grosor de la barra azul
                    backgroundColor: ColorSensorUp,
                }} />
                <div style={{
                    position: "absolute",
                    top: 0,
                    left: 0,
                    backgroundColor: "red",
                    height: 10, // Grosor de la barra azul
                    width: 40,
                }} />
            </div>
            <div style={{
                position: "absolute",
                bottom: "50%",
                height: sensorUp + 5, // Grosor de la barra azul
                fontSize: 15,
            }} > {sensorValue[0]}</div>


            {/* sensor s2 */}
            <div style={{
                top: "50%",
                position: "absolute",
                left: zeroPosX,
                height: sensorDown, // Grosor de la barra azul
                width: 22,
            }} >

                <div style={{
                    position: "absolute",
                    bottom: 0,
                    width: 20,
                    left: 10,
                    height: "100%", // Grosor de la barra azul
                    backgroundColor: ColorSensorDown,
                }} />
                <div style={{
                    position: "absolute",
                    bottom: 0,
                    left: 0,
                    backgroundColor: "red",
                    height: 10, // Grosor de la barra azul
                    width: 40,
                }} />
            </div>
            <div style={{
                position: "absolute",
                top: height / 2 + sensorDown - 15,
                left: 0,
                fontSize: 15,
            }} > {sensorValue[1]}</div>

            {/* altitude level sea*/}
            <div style={{
                position: "absolute",
                bottom: 10,
                left: zeroPosX + halfdrone * 2,
                fontSize: 15,
            }} > 120  ASL</div>
            {/* Indicador y Valor Actual */}
            <div
                style={{
                    position: "absolute",
                    color: indicatorColor,
                    left: zeroPosX + halfdrone * 2,
                    top: "50%",
                    transform: "translateY(-50%)",
                    width: 100,
                    height: 40,
                    display: "flex",
                    alignItems: "center"
                }}
            >
                <div
                    style={{
                        borderLeftColor: indicatorColor,
                        borderRight: " 15px solid ",
                        borderTop: " 10px solid transparent",
                        borderBottom: " 10px solid transparent"
                    }}
                />
                <div
                    style={{
                        backgroundColor: "white",
                        border: "1px solid #ccc",
                        padding: "5px 10px",
                        borderRadius: "4px",
                        whiteSpace: "nowrap",
                        fontSize: "18px",
                        fontWeight: "bold",
                        right: `${width + 5}px`,
                        color: labelColor,
                    }}
                >
                    {value.toFixed(1)} {unit}
                </div>
            </div>
        </div >
    );
};

export default LinearGauge;