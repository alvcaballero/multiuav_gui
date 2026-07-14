# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MultiUAV-GUI is a Ground Control Station (GCS) for commanding and monitoring heterogeneous fleets of UAVs for long-range inspection missions. The system integrates with ROS-based UAVs, supports real-time telemetry streaming, mission planning, video streaming, and LLM-powered assistance.

**Architecture**: Node.js/Express backend + React/Redux frontend + ROS bridge + WebSocket real-time communication

## Project Structure

| Directory     | Purpose                       | Local Docs             |
| ------------- | ----------------------------- | ---------------------- |
| `client/`     | Web UI (React 19/Vite/MUI v7) | `client/AGENTS.md`     |
| `server/`     | Backend server                | `server/AGENTS.md`     |


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


# Copy environment configuration
cp server/.env.example server/.env
# Edit server/.env with appropriate values

# Copy device configuration template
cp server/config/devices/.devices_init.yaml server/config/devices/devices_init.yaml
```

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

### Docker Setup

```bash
# Build ROS bridge container
cd docker
docker build -t muavgcs:noetic .

# Or build with non-root user
docker build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t muavgcs:noetic .

# Run all services with docker-compose
docker-compose up

# Run ROS bridge only
docker-compose run bridge roslaunch aerialcore_gui connect_uas.launch
```

### Tmuxinator (Optional)

```bash
# Launch full stack with tmuxinator
tmuxinator start -p muav-gui.yml
```

### Error Handling

- Server errors: Log via `logger.error()` and emit `SYSTEM_ERROR` event
- Client errors: Dispatch to `errors` slice, trigger snackbar + audio alert
- Service timeouts: 5-second default with error callbacks
