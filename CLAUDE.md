# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MultiUAV-GUI is a Ground Control Station (GCS) for commanding and monitoring heterogeneous fleets of UAVs for long-range inspection missions. The system integrates with ROS-based UAVs, supports real-time telemetry streaming, mission planning, video streaming, and LLM-powered assistance.

**Architecture**: Node.js/Express backend + React/Redux frontend + ROS bridge + WebSocket real-time communication

## Project Structure

| Directory | Purpose                                                     | Local Docs         |
| --------- | ----------------------------------------------------------- | ------------------ |
| `client/` | Web UI (React 19/Vite/MUI v7)                               | `client/CLAUDE.md` |
| `server/` | Backend server                                              | `server/CLAUDE.md` |
| `infra/`  | Cómo LEVANTARLO: compose, tmuxinator, `setup.sh`, guías     | `infra/README.md`  |
| `docs/`   | Decisiones, planes, referencia para personas, imágenes      | ver abajo          |

La raíz no lleva infraestructura (nada de `compose.yaml`, perfiles de tmuxinator ni `.env.example` sueltos): todo
eso va en `infra/`.

## Docs y backlog

Cada tipo de documento, en UN lugar:

| Qué | Dónde | Vida |
| --- | ----- | ---- |
| Qué falta (backlog) | `TODO.md` (raíz) | Cambia siempre. Es el ÚNICO |
| Por qué se hizo así | `docs/decisions/<tema>.md` | Plan cerrado: no cambia más |
| Plan en curso o propuesta sin decidir | `docs/plans/<tema>.md` | Temporal: se cierra cuando termina |
| Cómo funciona hoy (para Claude) | `CLAUDE.md` de cada carpeta | Se actualiza con el código |
| Cómo funciona hoy (para personas) | `docs/reference/` | Se actualiza con el código |
| Cómo levantarlo (docker, tmuxinator, SITL) | `infra/README.md`, `infra/guides/` (junto a sus scripts) | Se actualiza con la infra |
| Imágenes del README y de los docs | `docs/images/` | — |

Nombres de archivo en kebab-case (`vehicle-platforms.md`). Nada de `.md` sueltos en la raíz de `docs/`.

Reglas:
1. **Un solo backlog.** No crear otro `TODO*.md` ni listas de pendientes en otro archivo. Un pendiente sin plan
   activo va a `TODO.md`.
2. **Cerrar un plan es un solo commit**: sus pendientes pasan a `TODO.md` (verificados contra el código), el plan
   se mueve a `docs/decisions/` y queda solo con decisiones, mediciones e historial; lo que repite un `CLAUDE.md`
   se borra. El código cita el plan por sección (`docs/decisions/mavlink.md › F4`): al mover o recortar uno,
   `rg` el nombre y actualizar las referencias.
3. **Lo terminado se borra, no se tilda**: queda en `git log`.
4. **Nada de TODO gitignored**: si es privado, es una nota personal, no un backlog.
5. **Cómo levantar el proyecto se escribe UNA vez, en `infra/README.md`.** El README y los `CLAUDE.md` apuntan
   ahí, no lo copian: la copia es la que se pudre (pasó con `cd docker`, `docker-compose run bridge` y
   `muav-gui.yml`, que no existían).
6. **Ninguna ruta de una máquina concreta en un archivo versionado.** QUÉ pieza externa necesita el proyecto (repo,
   rama, versión) va en `infra/external.md` y `infra/repos/*.repos`; DÓNDE está en esta máquina, en
   `infra/local.env` (gitignored). Para encontrar PX4, ArduPilot, el workspace ROS, mqtt_client, psdk_ros2… leer
   `infra/local.env`; si no existe o le falta la variable, preguntar, no adivinar ni buscar por el disco.


## Development Commands

### Setup

```bash
# Clone repository with submodules
git clone --recursive https://github.com/alvcaballero/multiuav_gui.git
# Or if already cloned, initialize submodules
git submodule update --init --recursive

# Install server dependencies
cd server
npm install

# Install client dependencies
cd client
npm install

# server/.env, Docker images, submodules (from the repo root)
bash infra/setup.sh
```

Devices are not configured by file: they live in the DB and are created from the UI or `POST /api/devices`.

### Running the Application

```bash
# Start backend (from server/ directory)
npm run server          # Production mode on port 4000
npm run inspect         # Debug mode with Node inspector

# Start frontend development server (from client/ directory)
npm run start           # Development server on port 3000

# Build frontend for production (from client/ directory)
npm run build           # Creates optimized build in client/build/
```

### Testing

```bash
# Run server tests (from server/ directory)
npm test
```

### Code Quality (Client)

```bash
# From client/ directory
npm run lint            # Check for linting errors
npm run lint:fix        # Auto-fix linting errors
npm run format          # Format code with Prettier
npm run format:check    # Check formatting without changes
```

### Docker, tmuxinator, ROS bridge, planner, offline map

All in `infra/README.md` (rule 5 above).

### Error Handling

- Server errors: Log via `logger.error()` and emit `SYSTEM_ERROR` event
- Client errors: Dispatch to `errors` slice, trigger snackbar + audio alert
- Service timeouts: 5-second default with error callbacks
