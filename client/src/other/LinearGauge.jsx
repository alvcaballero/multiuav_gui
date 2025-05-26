import React, { useState, useEffect } from 'react';
import './LinearGauge.css'; // Importamos los estilos
import { mapIconKey, mapIcons, frontIcons } from '../Mapview/preloadImages';


const LinearGauge = ({
    value,                 // Valor actual a mostrar (ej: 10.0)
    sensorValue = [7, 5],
    sensorData = {up:7,down:5},
    sensorLimits = {up:[0,20],down:[0,20]},
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

    const setFillColor = (clampedDistance, limits=[0,10]) => {
        const limitLow = limits[0] + 0.3 * (limits[1] - limits[0])
        const limitHigh = limits[0] + 0.75 * (limits[1] - limits[0])
        if (clampedDistance  <= limitLow) return 'red'; // Distancia baja, seguro
        if (clampedDistance  <= limitHigh)   return 'yellow'; // Distancia media, precaución
        return'lightgray'; // Distancia alta, cerca
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
        const s1 = Math.max(sensorLimits.up[0], Math.min(sensorLimits.up[1], sensorValue[0]));
        const ratios1 = ( sensorLimits.up[1]- s1) / (sensorLimits.up[1] - sensorLimits.up[0]);
        setSensorUp(ratios1 *height / 2)
        setColorSensorUp(setFillColor(s1),sensorLimits.up)

        const  s2 = Math.max(sensorLimits.down[0], Math.min(sensorLimits.down[1], sensorValue[1]));
        const ratios2 = ( sensorLimits.down[1] -s2) / (sensorLimits.down[1] - sensorLimits.down[0]);
        setSensorDown(ratios2 * (height / 2) )
        setColorSensorDown(setFillColor(s2),sensorLimits.down)
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
            {/* sensor up */}
            <div style={{
                top: 0,
                position: "absolute",
                left: zeroPosX,
                width: 30,
                height: sensorUp, 
            }} >
                {/*  barra sensor*/}
                <div style={{
                    position: "absolute",
                    bottom: 0,
                    width: 20,
                    left: 10,
                    height: "100%", 
                    backgroundColor: ColorSensorUp,
                }} />
                {/*  linea de senalar*/}    
                <div style={{
                    position: "absolute",
                    bottom: 0,
                    left: 0,
                    backgroundColor: "red",
                    height: 10, 
                    width: 40,
                }} />
            </div>
            <div style={{
                position: "absolute",
                top: sensorUp -20,
                fontSize: 15,
            }} > {sensorValue[0]}</div>


            {/* sensor s2 */}
            <div style={{
                bottom: 0,
                position: "absolute",
                left: zeroPosX,
                height: sensorDown, 
                width: 22,
            }} >
                {/*  barra sensor*/}
                <div style={{
                    position: "absolute",
                    bottom: 0,
                    width: 20,
                    left: 10,
                    height: "100%", 
                    backgroundColor: ColorSensorDown,
                }} />
                {/*  linea de senalar*/}    
                <div style={{
                    position: "absolute",
                    top: 0,
                    left: 0,
                    backgroundColor: "red",
                    height: 10, 
                    width: 40,
                }} />
            </div>
            <div style={{
                position: "absolute",
                bottom: sensorDown - 15,
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