# Infraestructura — GCS MultiUAV

Este directorio documenta los repos externos y pasos necesarios para levantar
el entorno completo de la GCS.
El `compose.yaml` en la raíz del proyecto orquesta todos los servicios,
pero algunas imágenes Docker deben construirse desde sus propios repositorios.

---

## Setup inicial

```bash
bash infra/setup.sh
```

El script verifica prereqs, crea `server/.env` si no existe, y reporta
qué imágenes Docker faltan.

---

## Repositorios externos

### ROS Bridge

La imagen y el comando de arranque dependen de la versión de ROS.
Configurar en `.env` (copiar desde `.env.example`).

| Variable | ROS 1 (Noetic) | ROS 2 (Humble/Jazzy) |
|---|---|---|
| `ROSBRIDGE_IMAGE` | `muavgcs:ros1` | `muavgcs:ros2` |
| `ROSBRIDGE_CMD` | `roslaunch aerialcore_gui connect_uas.launch` | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |

**Construir la imagen ROS 1:**
```bash
# Desde el repo del workspace ROS (ver repo externo)
docker build -t muavgcs:ros1 -f Dockerfile .
```

**Construir la imagen ROS 2:**
```bash
# Desde el repo del workspace ROS2 (ver repo externo)
docker build -t muavgcs:ros2 -f Dockerfile .
```

> Los Dockerfiles de estas imágenes viven en sus respectivos repos del workspace ROS, no aquí.

### Planner (`muavgcs:planner`)

Servicio FastAPI con el solver MIP para planificación de misiones.

```bash
cd mip_planner
docker build -t muavgcs:planner .
```

> O arrancarlo sin Docker: `uv run uvicorn main:app --host 0.0.0.0 --port 8000 --reload`

---

## Servicios del compose

| Servicio      | Imagen                              | Puerto | Descripción                        |
|---------------|-------------------------------------|--------|------------------------------------|
| `node`        | `node:18`                           | 4000   | Backend Node.js/Express            |
| `glyphServer` | `node:12`                           | 8484   | Servidor de fuentes para el mapa   |
| `MapServer`   | `overv/openstreetmap-tile-server`   | 8080   | Tiles OSM (requiere volumen)       |
| `VideoServer` | `bluenviron/mediamtx`               | host   | Streaming WebRTC/RTSP              |
| `RosBridge`   | `$ROSBRIDGE_IMAGE` (ros1 o ros2)    | host   | Puente ROS → WebSocket (9090)      |
| `Planner`     | `muavgcs:planner`                   | 8004   | Solver MIP de misiones             |

---

## MapServer offline (opcional)

Para tener tiles OSM sin internet se necesita un volumen con datos precargados:

```bash
# Crear volumen e importar datos de la región (tarda bastante)
docker volume create osm-data-andalucia
docker run -v osm-data-andalucia:/data/database \
  overv/openstreetmap-tile-server \
  import
# Copiar el .pbf de la región antes de importar
```

Sin el volumen el MapServer arranca pero no servirá tiles.

---

## Arrancar la stack

```bash
# Primera vez
bash infra/setup.sh

# Arrancar todo
docker compose up

# Solo algunos servicios (ej: sin MapServer offline)
docker compose up node VideoServer RosBridge Planner
```
