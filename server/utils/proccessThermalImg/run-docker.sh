#!/bin/bash
#
# Docker wrapper for processThermalGen.py
#
# Lets ProcessFile.js keep invoking a flat "<cmd> -i <input> -o <output>" while
# this script translates the host paths into a Docker bind mount. It mounts the
# PARENT DIRECTORY of the images as /data (opción A) so we avoid the
# "output file mounted as a directory" trap and only need a single -v.
#
# Requirements:
#   - Image built once:  docker build -t muavgcs:processImage .
#   - input and output MUST live in the same directory.
#
# Usage (same interface as the python script):
#   ./run-docker.sh -i /abs/path/img.jpg -o /abs/path/img_process.jpg
#
set -euo pipefail

IMAGE="${THERMAL_DOCKER_IMAGE:-muavgcs:processImage}"

input=""
output=""

# Parse the same flags the python script accepts (-i / -o).
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--ifile|--source)
      input="$2"
      shift 2
      ;;
    -o|--ofile|--dest)
      output="$2"
      shift 2
      ;;
    *)
      echo "run-docker.sh: unknown argument '$1'" >&2
      exit 2
      ;;
  esac
done

if [[ -z "$input" || -z "$output" ]]; then
  echo "run-docker.sh: -i <input> and -o <output> are required" >&2
  exit 2
fi

if [[ ! -f "$input" ]]; then
  echo "run-docker.sh: input file does not exist: $input" >&2
  exit 2
fi

# Resolve to absolute paths so the bind mount is unambiguous.
input="$(realpath "$input")"
in_dir="$(dirname "$input")"
in_name="$(basename "$input")"

# The output may not exist yet; resolve its directory via its parent.
out_dir="$(realpath "$(dirname "$output")")"
out_name="$(basename "$output")"

# Opción A needs both files under the same directory (single -v mount).
if [[ "$in_dir" != "$out_dir" ]]; then
  echo "run-docker.sh: input and output must be in the same directory (opción A)." >&2
  echo "  input dir : $in_dir" >&2
  echo "  output dir: $out_dir" >&2
  exit 2
fi

# The container MUST run as root, and the CWD MUST stay /usr/src/app:
# thermal_base invokes the DJI SDK via relative paths ("./dji_executables/...",
# LD_LIBRARY_PATH, and a temporary "output.raw") all resolved against the CWD,
# and that directory is owned by root. Running as a non-root --user therefore
# fails (the SDK can't write output.raw there), so --user is not an option.
#
# Instead we override the entrypoint: run the script as root, then chown the
# generated output back to the host user from INSIDE the container (root can
# chown to any uid). This way the Node server — running as the host user — can
# move/delete the file afterwards. HOST_UID/HOST_GID are read inside the shell.
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

exec docker run --rm \
  -e "HOST_UID=${HOST_UID}" \
  -e "HOST_GID=${HOST_GID}" \
  -v "${in_dir}:/data" \
  --entrypoint /bin/sh \
  "$IMAGE" \
  -c 'python ./processThermalGen.py -i "$1" -o "$2"; status=$?; [ -f "$2" ] && chown "${HOST_UID}:${HOST_GID}" "$2"; exit $status' \
  _ "/data/${in_name}" "/data/${out_name}"
