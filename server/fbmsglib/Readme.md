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
  /opt/local/bin/flatc --ts --gen-all --gen-object-api -o ./src ./messages/schema_main.fbs
```

`--gen-object-api` adds a `<Name>T` class per table with `unpack()` / `pack()` (a buffer →
plain object, used by the raw message inspector). With flatc 25.x `--gen-onefile` no longer
produces a single file, so it is not passed: the output is `src/schema_main.ts` + `src/fb/`,
which tsup bundles into the one `dist/schema_main.cjs`. `src/` is not tracked; `dist/` is.
The generated code works with the `flatbuffers` runtime of the server (24.12.x).

## Compile src → dist

```bash
# Run from server/
npm run build:fbmsglib
```
## compile  from build for c++
```
./flatbuffers/flatc --cpp --gen-all -n  ./messages/schema_main.fbs
```