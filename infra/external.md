# Piezas externas

QUÉ necesita el GCS de fuera de este repo: repo, versión y para qué. DÓNDE está cada una en tu máquina NO va
acá: va en `infra/local.env` (gitignored, plantilla en `infra/local.env.example`), que leen los perfiles de
tmuxinator, compose (`--env-file infra/local.env`) y Claude.

Los workspaces ROS se reconstruyen con [vcstool](https://github.com/dirk-thomas/vcstool) (`pip install vcstool`):

```bash
vcs import $ROS2_WS/src < infra/repos/ros2_px4.repos      # lado dron, ROS 2 (PX4, DJI PSDK, MQTT)
vcs import <catkin_ws>/src < infra/repos/ros1_noetic.repos  # ROS bridge ROS 1 (DJI OSDK, robofleet)
```

La versión que figura acá es la que el GCS espera: si un workspace tiene otra, el que está mal es el workspace.

## Mensajes ROS que usan las interfaces del GCS

Cada `messageType` de `server/config/devices/interfaces/` sale de uno de estos paquetes. Sin el paquete compilado
en el lado ROS, rosbridge no puede (de)serializar el mensaje.

| Paquete | Repo | Versión | Interfaces del GCS |
|---|---|---|---|
| `px4_msgs` | [PX4/px4_msgs](https://github.com/PX4/px4_msgs) | `main` (= PX4 1.18) | `px4_ros2`. Usa topics versionados (`vehicle_status_v4`, `battery_status_v1`), que NO existen en `release/1.15` |
| `psdk_interfaces` | [arpoma16/psdk_ros2](https://github.com/arpoma16/psdk_ros2) (fork de umdlife) | `dev/integration` | `dji_M300_PSDK` |
| `muav_gcs_interfaces` | [arpoma16/MUAV_GCS_interfaces](https://github.com/arpoma16/MUAV_GCS_interfaces) | `main` | `px4_ros2`, PSDK |
| `dji_osdk_ros` | [miggilcas/Onboard-SDK-ROS](https://github.com/miggilcas/Onboard-SDK-ROS) | `dev` | `dji_M210_*`, `dji_M300` (ROS 1). Necesita [DJI Onboard-SDK](https://github.com/dji-sdk/Onboard-SDK) `master` instalado (cmake, no catkin) |
| `aerialcore_common` | [alvcaballero/multiUAV_system](https://github.com/alvcaballero/multiUAV_system) | `main` | DJI ROS 1 (misiones). Trae `aerialcore_gui` (`connect_uas.launch`) |
| `mavros_msgs` | apt (`ros-noetic-mavros-msgs`) | — | `px4_mavros`, `griffin` |
| `nav2_msgs` | apt (`ros-<distro>-nav2-msgs`) | — | UGV con Nav2 |

`sensor_msgs`, `std_msgs`, `geometry_msgs`, `std_srvs`, `nav_msgs` vienen con ROS. `mavlink/*` no es ROS: es
`node-mavlink` (npm, en el server).

### Versiones de mensajes de PX4: una sola interfaz + translation node

Desde PX4 1.16 cada mensaje versiona por su cuenta y el topic lleva la versión (`<topic>_v<N>`; versión 0 = sin
sufijo, como en 1.15). Con el PX4 que espera el GCS (`main`, 1.18): `vehicle_status_v4`; `battery_status`,
`vehicle_command_ack`, `vehicle_local_position` en `_v1`; `vehicle_global_position` y `vehicle_command` sin sufijo.

**Decisión: el GCS fija UNA versión de mensajes y NO hay una interfaz por versión de PX4.** rosbridge deserializa
con el único `px4_msgs` que tiene compilado; un `VehicleStatus` de 1.15 y uno de 1.18 tienen el mismo nombre de
tipo y estructuras distintas, así que interfaces separadas no harían convivir drones de versiones distintas.

- La interfaz `px4_ros2` y el `px4_msgs` del rosbridge van en `main` (tabla de arriba).
- Cada dron con otro PX4 corre el **message translation node** de PX4
  ([docs](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html), `PX4-Autopilot/msg/translation_node`):
  traduce cada topic entre la versión que publica el firmware y la que espera el GCS (v0↔v1↔…↔v4), 1.15 incluido.
- **Uno por dron**: resuelve los topics con su propio namespace (`get_effective_namespace()`), así que va con el
  namespace del dron, al lado del Agent XRCE-DDS en su companion:
  ```bash
  cd <ws> && $PX4_DIR/Tools/copy_to_ros_ws.sh . && colcon build && source install/setup.bash
  ros2 run translation_node translation_node_bin --ros-args -r __ns:=/<device>
  ```
- Compilarlo desde un PX4 **igual o más nuevo** que el más nuevo de los dos lados (firmware o GCS): solo traduce las
  versiones que conoce (`msg/px4_msgs_old/`).
- **Subir de versión** (sale un PX4 nuevo): se actualizan una sola vez `px4_msgs` y los topics de `px4_ros2`, y se
  recompila el translation node en los drones. Los firmwares viejos de la flota no se tocan.
- Límites (docs de PX4): es **experimental**; la traducción de **servicios no funciona en Humble** (topics sí; el
  GCS por `px4_ros2` solo usa topics); al traducir a una versión más nueva, los campos que la vieja no tenía
  llegan con su default, no con un error.
- Una interfaz aparte (`extends: px4_ros2`, solo los nombres de topic) queda como último recurso para un dron que
  no pueda correr el nodo, y arrastra el problema del `px4_msgs` del rosbridge.

## Transportes alternativos a rosbridge (a bordo)

| Pieza | Repo | Versión | Para qué |
|---|---|---|---|
| mqtt_client | [arpoma16/mqtt_client](https://github.com/arpoma16/mqtt_client) (fork de [ika-rwth-aachen](https://github.com/ika-rwth-aachen/mqtt_client)) | `muav/service-bridge` | Protocolo `mqtt`: telemetría + bridge de servicios (`docs/decisions/mqtt.md`). El broker corre en el GCS: `infra/compose/compose.mqtt.yaml` |
| robofleet_client | [arpoma16/robofleet_client](https://github.com/arpoma16/robofleet_client) | `master` | Protocolo `robofleet` (FlatBuffers, ROS 1). Los `.fbs` del GCS están en `server/fbmsglib/` |
| Micro-XRCE-DDS Agent | [eProsima/Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) | `v2.4.3` | Puente uORB ↔ ROS 2 de PX4 (`px4_ros2`) |

## Autopilotos y simuladores

| Pieza | Repo | Versión | Para qué |
|---|---|---|---|
| PX4-Autopilot | [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | `main` (1.18) | SITL (`make px4_sitl gz_x500`, `gz_standard_vtol`) y leer el comportamiento real (navigator, `FeasibilityChecker.cpp`, ROMFS). `guides/sitl-px4.md` |
| ArduPilot | [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) | tag por vehículo: `Copter-4.7.2`, `Rover-4.7.2` (en 4.7.2 son el mismo commit) | SITL por MAVLink. El vehículo se elige al compilar, no con el checkout: `infra/docker/ardupilot-sitl/Dockerfile` (`AP_TAG`, `AP_VEHICLE`) |
| sjtu_drone | [arpoma16/sjtu_drone](https://github.com/arpoma16/sjtu_drone) | `ros2` | Quadrotor de Gazebo sin autopiloto (interfaz `sjtu`, sin verificar: ver `TODO.md`) |

## Servicios que el GCS consume

| Pieza | Repo | Versión | Cómo se levanta |
|---|---|---|---|
| Smart-Flies-MUAVPlanner | submódulo hermano de `llm_planner_gcs` | el puntero del submódulo | `compose.planner.yaml` o `gcs-full.yml` |
| mcp_server | submódulo hermano de `llm_planner_gcs` | el puntero del submódulo | `compose.mcp.yaml` o `gcs-mcp.yml` |
| glyphserver | [Geodan/glyphserver](https://github.com/Geodan/glyphserver) | `master` | `compose.map.yaml` (`GLYPHSERVER_DIR`) |
| Imagen ROS bridge `muavgcs:ros1` | Dockerfile del workspace ROS 1 | — | `compose.ros1.yaml`. Lo que lleva: `guides/ros1-workspace.md` |
