======================
VPN con Fuvex
======================

sudo wg-quick up wg0
sudo ip link set wg0 down
sudo ip link set wg0 up
sudo wg (para Comprobar)
(ping 10.20.20.1)


=======================
Simulación DJI - hexarotor
=======================

En el dron Dji:
    0. Hacer ping entre equipos (verificar que archivo hosts de drones y pc están bien registradas las ips)
    1. Lanzar Roscore
    2. Lanzar alias (M210-sim-on)
    3. Lanzar dji_vehicle_node (roslaunch aerialcore_onboard_dji atlas.launch)
    5. roslaunch aerialcore_onboard_dji multimaster.launch (y comprobar en el pc que ve los rostopics del dron)
    

En el PC (GCS):
    4. Lanzar rosbridge_server (roslaunch aerialcore_gui connect_uas.launch).
    6. Lanzar GUI, conectar Ros, añadir dron
    7. Cargarle y lanzar la mision

=======================
Simulación PX4 - VTOL
=======================

En el pc:
    Lanzar por consola la QGroundControl (./QGroundControl.AppImage)
    Lanzar rosbridge_server (connect_uas.launch)
    Lanzar GUI (desde la carpeta "aerialcore_gui" -> npm run dev), conectar Ros, añadir dron
    Lanzar en aerialcore_onboard_px4 el simulation.launch ns:="uav_3" (o el ns correspondiente)
    Cargarle y lanzar la mision

==========================================
Multimaster requirements
==========================================
Dron:

roscore
roslaunch enel_onboard_dji atlas.launch
roslaunch enel_onboard_dji multimaster.launch

pc:

roslaunch aerialcore_gui connect_uas.launch

==========================================
PX4 Real Flight
==========================================

- Conectar antena al pc en puerto serie)
- en terminal aparte: roslaunch aerialcore_onboard_px4 atlas.launch
- en terminal aparte: roslaunch aerialcore_gui connect_uas.launch
- Lanzar GUI (desde la carpeta "aerialcore_gui" -> npm run dev), conectar Ros, añadir dron
Revisar bien los uav_id que coincidan con los correspondientes en la mision.
- Cargar mision
- Lanzar mision



[El dron estaba conectado por antena radio al pc de Arturo. En el pc de Arturo se corría el atlas.launch y el multimaster, haciendose pasar por el pc de abordo del dron. Nosotros vamos a replicar este proceso pero en la misma GCS]
Si en lugar de este metodo se quisiera utilizar un pc de abordo, habría que cambiar el baudrate de comunicacion

===================
SETUP COMPLETO
===================


DRON DJI (aterrizaje en estacion de carga) - uav_1

    Ricardo y Batista lo controlan (Similar al DJI uav_2)

DRON DJI - uav_2

    0. Hacer ping entre equipos (verificar que archivo hosts de drones y pc están bien registradas las ips)
    1. Lanzar Roscore
    2. Lanzar alias (M210-sim-on) (o real si no es simulacion)
    3. Lanzar dji_vehicle_node (roslaunch aerialcore_onboard_dji atlas.launch)
    4. roslaunch aerialcore_onboard_dji multimaster.launch (y comprobar en el pc que ve los rostopics del dron)
    

VTOL - uav_3

    Se lanza en la GCS. Comprobar con la QGround que se reciben las misiones correctamente. En el archivo atlas.launch situado en la carpeta aerialcore_onboard_px4 se puede modificar la ip a la que se envia la telemetria (pc de Victor. Abrir ahi la Qground)

MARVIN FUVEX - uav_4

    Depende de Fuvex. SU GCS ha de conectarse al puerto 3000 de la GCS USE (éste pc) para conectar los websockets

PC - GCS

    Conectar antena al pc en puerto serie (receptor del VTOL)
    Lanzar roslaunch aerialcore_onboard_px4 atlas.launch
    Lanzar rosbridge_server (roslaunch aerialcore_gui connect_uas.launch).
    Lanzar GUI (desde la carpeta "aerialcore_gui" -> npm run dev)
    - conectar Ros
    - añadir drones
    Revisar bien los uav_id que coincidan con los correspondientes en la mision.
    - Cargar mision
    - Lanzar mision
