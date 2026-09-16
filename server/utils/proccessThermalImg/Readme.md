# processThermalImg

Procesa las imágenes térmicas de las misiones DJI (M300 / H20T) y WirisPro:
decodifica la matriz de temperatura, marca el punto de temperatura máxima y
escribe un JPG anotado. Además inserta la temperatura máxima en el campo EXIF
`UserComment` del archivo de salida.

El servidor lo invoca desde [`server/models/ProcessFile.js`](../../models/ProcessFile.js)
(`ProcessThermalImage`) con la interfaz:

```bash
<comando> -i <input.jpg> -o <output.jpg>
```

Qué comando se usa lo define la variable `PROCESS_THERMAL_IMG_SRC` del `.env`.
Hay **dos formas de ejecutarlo**: `uv` (local) o Docker (contenedor). Ambas
respetan la misma interfaz `-i/-o`.

Activar/desactivar el procesamiento: `PROCESS_THERMAL_IMG=true|false` en el `.env`.

---

## Requisitos comunes

El script depende del **DJI Thermal SDK** (`dji_irp`) para decodificar el RAW
térmico DJI, y del binario `exiftool` del sistema. En la imagen Docker ambos
vienen incluidos; en local `thermal_base` descarga el SDK y `exiftool` debe estar
instalado en el host.

---

## Opción 1 — Ejecución local con `uv` (recomendada para el server nativo)

El proyecto Python vive en este mismo directorio (`pyproject.toml`). `uv` resuelve
el entorno (`.venv`) con todas las dependencias de `requirements.txt`.

**`.env`:**

```dotenv
PROCESS_THERMAL_IMG=true
# Borrar PROCESS_THERMAL_IMG_SRC para usar el default del código, que ya resuelve
# la ruta absoluta con --project y no depende del CWD. Ver server/config/config.js
```

El default que arma [`config.js`](../../config/config.js) equivale a:

```bash
uv run --project <dir> <dir>/processThermalGen.py -i <input> -o <output>
```

> ℹ️ La primera ejecución descarga ~200 MB de dependencias (opencv, `thermal_base`
> desde git) y las cachea en `.venv`. Las siguientes son inmediatas.

**Probar a mano:**

```bash
cd server
uv run --project ./utils/proccessThermalImg \
  ./utils/proccessThermalImg/processThermalGen.py \
  -i /ruta/absoluta/DJI_XXXX_THRM.jpg \
  -o /ruta/absoluta/DJI_XXXX_THRM_process.jpg
```

> ⚠️ Usar rutas **absolutas**. El script detecta el formato con
> `inputfile.split(".")[1]`, que se rompe con rutas relativas tipo `./...`.

Requisitos: `uv` y `exiftool` instalados en el host. Agregar `.venv/` al
`.gitignore` para no trackear el entorno.

---

## Opción 2 — Ejecución con Docker (wrapper `run-docker.sh`)

Útil cuando el host no tiene `uv`/Python o se prefiere el entorno encapsulado.
El wrapper [`run-docker.sh`](./run-docker.sh) mantiene la interfaz `-i/-o` y por
dentro traduce las rutas a un bind mount.

**Buildear la imagen (una sola vez):**

```bash
cd server/utils/proccessThermalImg
docker build -t muavgcs:processImage .
```

**`.env`:**

```dotenv
PROCESS_THERMAL_IMG=true
PROCESS_THERMAL_IMG_SRC='./utils/proccessThermalImg/run-docker.sh'
# o la ruta absoluta si no arrancás el server desde server/
```

**Probar a mano:**

```bash
./utils/proccessThermalImg/run-docker.sh \
  -i /ruta/DJI_XXXX_THRM.jpg \
  -o /ruta/DJI_XXXX_THRM_process.jpg
```

**Uso directo del contenedor (sin wrapper), como referencia:**

```bash
docker run --rm \
  -v /ruta:/data \
  muavgcs:processImage \
  -i /data/DJI_XXXX_THRM.jpg -o /data/DJI_XXXX_THRM_process.jpg
```

### Cómo funciona el wrapper (y por qué así)

- **Monta el directorio padre** de las imágenes como `/data` (un solo `-v`).
  Por eso **input y output deben estar en el mismo directorio** — en el server
  siempre lo están (`.../uav_N/DJI_..._THRM.jpg` y `..._THRM_process.jpg`).
- **Corre como root** (no usa `--user`): `thermal_base`/`dji_irp` escriben un
  temporal `output.raw` con rutas relativas al CWD `/usr/src/app`, que es
  propiedad de root; ejecutar como usuario no-root falla ahí.
- **Devuelve la propiedad del output al usuario del host** haciendo `chown`
  desde dentro del contenedor (root puede chownear a cualquier UID), para que
  el server (que corre como el usuario del host) pueda mover/borrar el archivo.
- Variables:
  - `THERMAL_DOCKER_IMAGE` — nombre de la imagen (default `muavgcs:processImage`).

Requisitos: Docker instalado y el usuario del server con acceso al daemon
(grupo `docker`).

---

## Configuración usada por el servidor

| Variable                  | Descripción                                                        |
| ------------------------- | ------------------------------------------------------------------ |
| `PROCESS_THERMAL_IMG`     | `true`/`false` — habilita el procesamiento térmico                 |
| `PROCESS_THERMAL_IMG_SRC` | Comando a ejecutar. Vacío → default `uv` de `config.js`. O `run-docker.sh` |
