#!/usr/bin/env bash
# setup.sh — prepara el entorno completo de la GCS
# Ejecutar UNA vez antes de arrancar por primera vez.
# Uso: bash infra/setup.sh

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()    { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ─── 1. Prereqs ────────────────────────────────────────────────────────────────

info "Verificando prereqs..."

command -v docker  >/dev/null 2>&1 || error "Docker no instalado. Ver https://docs.docker.com/engine/install/"
command -v git     >/dev/null 2>&1 || error "Git no instalado."

DOCKER_COMPOSE_OK=false
docker compose version >/dev/null 2>&1 && DOCKER_COMPOSE_OK=true
command -v docker-compose >/dev/null 2>&1 && DOCKER_COMPOSE_OK=true
$DOCKER_COMPOSE_OK || error "Docker Compose no encontrado (ni plugin ni standalone)."

info "Prereqs OK."

# ─── 2. Variables de entorno ───────────────────────────────────────────────────

if [ ! -f server/.env ]; then
  if [ -f server/.env.example ]; then
    cp server/.env.example server/.env
    warn "Creado server/.env desde .env.example — revisá las variables antes de arrancar."
  else
    warn "No existe server/.env ni server/.env.example — crealo manualmente."
  fi
else
  info "server/.env ya existe, no se sobreescribe."
fi

# ─── 3. Imágenes Docker ────────────────────────────────────────────────────────

info "Verificando imágenes Docker necesarias..."

check_image() {
  local image="$1"
  local repo_hint="$2"
  if docker image inspect "$image" >/dev/null 2>&1; then
    info "  ✓ $image"
  else
    warn "  ✗ $image — no encontrada. Construila desde: $repo_hint"
  fi
}

# muavgcs:ros1 viene del repo del ROS workspace (ver infra/README.md)
check_image "muavgcs:ros1"     "Ver infra/README.md → ROS Bridge"
check_image "muavgcs:planner"  "Ver infra/README.md → Planner"

# ─── 4. Volumen OSM (opcional) ────────────────────────────────────────────────

if docker volume inspect osm-data-andalucia >/dev/null 2>&1; then
  info "Volumen OSM osm-data-andalucia existe."
else
  warn "Volumen osm-data-andalucia no existe. MapServer arrancará pero sin datos offline."
  warn "Para crearlo con datos: ver infra/README.md → MapServer offline."
fi

# ─── 5. Submodules ────────────────────────────────────────────────────────────

if [ -f .gitmodules ]; then
  info "Inicializando submodules..."
  git submodule update --init --recursive
fi

# ─── Listo ─────────────────────────────────────────────────────────────────────

echo ""
info "Setup completo. Para arrancar la stack:"
echo ""
echo "    docker compose up"
echo ""
warn "Si falta alguna imagen Docker, seguí los pasos en infra/README.md antes de arrancar."
