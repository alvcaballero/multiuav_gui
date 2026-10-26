# Infraestructura — GCS MultiUAV

Todo lo necesario para LEVANTAR el GCS vive en esta carpeta, y en ningún otro lado:

| Qué | Dónde |
|---|---|
| Setup inicial (prereqs, `server/.env`, imágenes, submódulos) | `setup.sh` |
| Piezas externas: QUÉ repo y versión necesita el GCS | `external.md` |
| Piezas externas: DÓNDE están en tu máquina (gitignored) | `local.env` (plantilla `local.env.example`) |
| Workspaces ROS reproducibles (vcstool) | `repos/*.repos` |
| Servicios en Docker, uno por archivo | `compose/` |
| Imágenes propias (ArduPilot SITL) | `docker/` |
| Sesiones de desarrollo (GCS con Node/npm, el resto en Docker) | `tmuxinator/` |
| Guías de piezas externas (cámaras, mapa offline, SITL) | `guides/` |

Cómo funciona el GCS (no cómo se levanta) está en los `CLAUDE.md` y en `docs/`.

## Primera vez en una máquina

```bash
bash infra/setup.sh
cp infra/local.env.example infra/local.env      # y poné tus rutas
vcs import <ROS2_WS>/src < infra/repos/ros2_px4.repos   # lado dron PX4 (Jazzy), si corre en esta máquina
# DJI PSDK va en su propio workspace Humble: infra/repos/ros2_psdk_humble.repos (infra/external.md)
```

Ninguna ruta de una máquina concreta va en un archivo versionado: va en `infra/local.env`.

---

## Setup inicial

```bash
bash infra/setup.sh
```

Verifica prereqs, crea `server/.env` desde `server/.env.example` si no existe, reporta qué imágenes Docker faltan e
inicializa los submódulos.

---

## Repositorios externos

### ROS Bridge (`muavgcs:ros1` / `muavgcs:ros2`)

Las imágenes se construyen desde sus repos del workspace ROS, no desde acá:

```bash
docker build -t muavgcs:ros1 -f Dockerfile .   # desde el repo del workspace ROS 1
docker build -t muavgcs:ros2 -f Dockerfile .   # desde el repo del workspace ROS 2
```

Qué lleva el workspace ROS 1 (DJI OSDK, multiUAV_system) y el puerto en WSL: `guides/ros1-workspace.md`.

Se elige ROS 1 o ROS 2 con el archivo de compose (`compose.ros1.yaml` / `compose.ros2.yaml`), no con variables.

### Planner (`muavgcs:planner`)

Es **Smart-Flies-MUAVPlanner** (Flask), el planner al que el GCS le pide misiones (`/mission_request`,
`/get_plan`; `PLANNING_SERVER=true`, `PLANNING_HOST=http://127.0.0.1:8004` en `server/.env`). Es el submódulo
hermano de `llm_planner_gcs`:

```bash
cd ../Smart-Flies-MUAVPlanner && docker build -t muavgcs:planner .   # escucha en 8080, compose lo expone en 8004
# sin Docker:
cd ../Smart-Flies-MUAVPlanner && uv run python -m flask --app ./api run --host=0.0.0.0 --port=8004
```

`mip_planner` NO es este servicio: lo usa solo `llm_mission_evaluator` (ver `mip_planner/CLAUDE.md`).

### MCP server y glyphserver

- MCP: submódulo hermano `llm_planner_gcs/mcp_server`. Sin imagen propia: compose monta su código en `node:22`.
- glyphserver ([Geodan/glyphserver](https://github.com/Geodan/glyphserver)): fuentes para el mapa. Se busca al
  lado de `llm_planner_gcs`; `GLYPHSERVER_DIR` para otro lugar.

---

## Compose — un archivo por servicio

Se combinan con `-f`. `compose.base.yaml` va siempre primero.

| Archivo | Servicios | Puerto | Cuándo |
|---|---|---|---|
| `compose.base.yaml` | `node` (`node:20`), `VideoServer` (mediamtx) | 4000, host | Siempre |
| `compose.ros1.yaml` | `RosBridge` (Noetic) | 9090 (host) | UAVs con ROS 1 |
| `compose.ros2.yaml` | `RosBridge` (Humble/Jazzy) | 9090 (host) | UAVs con ROS 2 |
| `compose.planner.yaml` | `Planner` (Smart-Flies) | 8004 | `PLANNING_SERVER=true` |
| `compose.mcp.yaml` | `MCP` | 3001 (host) | `MCP_ENABLE=true` + LLM |
| `compose.map.yaml` | `MapServer` (OSM), `glyphServer` | 8080, 8484 | Mapa offline |
| `compose.mqtt.yaml` | `Mosquitto` | 1883 (host, solo localhost) | `MQTT_CONNECTION=true` |

```bash
C=infra/compose

# GCS mínima (sin ROS, sin mapa offline)
docker compose -f $C/compose.base.yaml up

# + ROS 1
docker compose -f $C/compose.base.yaml -f $C/compose.ros1.yaml up

# Todo: ROS 1 + planner + MCP + mapa offline
docker compose -f $C/compose.base.yaml -f $C/compose.ros1.yaml -f $C/compose.planner.yaml \
  -f $C/compose.mcp.yaml -f $C/compose.map.yaml up
```

> **Variables de Compose:** las `${VAR}` de estos archivos (`GLYPHSERVER_DIR`, `ROS1_IMAGE`) salen de tus rutas
> con `docker compose --env-file infra/local.env -f ...`. Sin `--env-file` se toman del shell o de
> `infra/compose/.env` (la carpeta del primer `-f`); un `.env` en la raíz del repo NO se lee. No confundir con
> `server/.env`, que es la configuración del server.

### ArduPilot SITL

ArduPilot es un solo repo con un tag por vehículo (`Copter-4.7.2`, `Rover-4.7.2`); el vehículo se elige al
compilar. Una imagen por vehículo desde el mismo Dockerfile:

```bash
docker build -t muav-arducopter-sitl:4.7.2 infra/docker/ardupilot-sitl
docker build -t muav-ardurover-sitl:4.7.2 --build-arg AP_TAG=Rover-4.7.2 --build-arg AP_VEHICLE=rover \
  infra/docker/ardupilot-sitl

# SYSID = attributes.systemId del device en el GCS
docker run --rm --network=host -e SYSID=2 -e GCS_OUT=udpclient:127.0.0.1:14550 muav-arducopter-sitl:4.7.2
```

### MapServer offline

Necesita un volumen con los datos de la región ya importados (tarda bastante):

```bash
docker volume create osm-data-andalucia
docker run -v /ruta/region.osm.pbf:/data/region.osm.pbf -v osm-data-andalucia:/data/database \
  overv/openstreetmap-tile-server import
```

Sin el volumen el MapServer arranca pero no sirve tiles. Alternativa con martin + planetiler:
`guides/offline-map.md`.

---

## Tmuxinator — desarrollo local

El GCS corre con Node/npm (`nvm use` lee el `.nvmrc` de cada carpeta) y el resto en Docker. Los perfiles
encuentran el repo con `git rev-parse`, así que se arrancan desde cualquier carpeta del repo (`GCS_DIR` lo
pisa).

| Perfil | Qué levanta |
|---|---|
| `gcs-simple.yml` | backend + frontend, sin ROS ni Docker |
| `gcs-ros1.yml` | + ROS Bridge Noetic (compose) |
| `gcs-mcp.yml` | + MCP server (flujo LLM); `MCP_DIR` para otro lugar |
| `gcs-dev.yml` | GCS + MCP + ROS 1, planner y video en compose + rosbag |
| `gcs-full.yml` | El de laburo diario: GCS + glyphserver + Smart-Flies sin Docker + ROS 1 por `container_run.sh` + mediamtx + mapa con martin. Lo de fuera del repo sale de `infra/local.env` |

```bash
tmuxinator start -p infra/tmuxinator/gcs-simple.yml
```

---

## Guías

| Guía | Tema |
|---|---|
| `guides/ros1-workspace.md` | Workspace catkin del ROS bridge (DJI OSDK, multiUAV_system), WSL |
| `guides/camera-stream.md` | GStreamer y stream de cámara del dron |
| `guides/offline-map.md` | Tiles OSM offline, glyphs, martin + planetiler |
| `guides/sitl-px4.md` | PX4 SITL con Gazebo en Docker |
