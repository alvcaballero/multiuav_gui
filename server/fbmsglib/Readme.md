# fbmsglib

FlatBuffers message library for ROS message types used in the UAV GCS.
Flatbuffers repo: https://github.com/google/flatbuffers
Reference: https://flatbuffers.dev/tutorial/

## Structure

```
messages/   # FlatBuffers schemas (.fbs) — source of truth
src/        # Generated TypeScript (do not edit manually)
dist/       # Compiled output (.cjs) — consumed by the server
```

## Regenerate from schemas

Requires Docker:

```bash
# Run from fbmsglib/
docker run --rm \
  -u $(id -u):$(id -g) \
  -v $(pwd):/app -w /app \
  sptrakesh/flatbuffers:latest \
  /opt/local/bin/flatc --ts --gen-all --gen-onefile -o ./src ./messages/schema_main.fbs
```

## Compile src → dist

```bash
# Run from server/
npm run build:fbmsglib
```
## compile  from build for c++
```
./flatbuffers/flatc --cpp --gen-all -n  ./messages/schema_main.fbs
```