# AGENTS.md

Two independent sub-projects share this repo. No tests, CI, or lint exist.

## Web map (`src/`, TypeScript + MapLibre)

- Dev command: `npm start` (runs `parcel watch src/index.html --out-dir ./example/dev-bundle/ --public-url . --no-cache` together with `static-server`). The server comes up on port 9080, matching `base_url` (`src/index.ts:3`).
- `example/` is gitignored build output. The map hardcodes `base_url = "http://localhost:9080/example/dev-bundle"` (`src/index.ts:3`) and fetches `data/starting_points.geojson` and `data/isochrones/{h3Index}.geojson` from there. Nothing renders until you copy the C++ tool's output (in `data/`) into `example/dev-bundle/data/`.
- `src/index.ts` contains a hardcoded Maptiler API key and style URL — don't add more keys.
- Old toolchain: parcel-bundler v1, maplibre-gl v2, TypeScript 4.9.

## Isochrone generator (`gtfs-isochrones/`, C++20)

- Computes isochrones on an H3 resolution-10 grid: edges from GTFS trips (worst-case wait + travel time) plus OSM road walking at 5 km/h. Only Wednesday services at 12:35 are kept (`check_time = 3600*12+60*35`, `Isochrone_generator.cpp:1007`).
- Build via the Xcode project (`gtfs-isochrones.xcodeproj`, searches `/opt/homebrew/include` and `/opt/homebrew/lib`) or the CMakeLists.txt. macOS-only (includes `<mach/mach.h>`). Dependencies come from Homebrew: GDAL, H3, libosmium, nlohmann-json.
- `main.cpp` uses repo-relative paths: inputs `data/gtfs` and `data/osm/mexico-260423.osm.pbf`, outputs `data/starting_points.geojson` and `data/isochrones/`. Run the binary from the repo root (the `build/` dir has a cmake build). After a run, copy those outputs into `example/dev-bundle/data/` for the map.
- Polygon merging uses H3 `cellsToLinkedMultiPolygon` (holes preserved); GDAL is only used by the debug gpkg writers.
- Output: `starting_points.geojson` plus one `{h3Index}.geojson` per stop; `properties.id` is the H3 index string the map fetches by (`src/index.ts:199`).
- GTFS parsing is hand-rolled CSV with strict column counts; stops parsing has a special case for quoted comma-containing stop names (Mexico City data). A row with unexpected column count aborts with "Problems parsing ...".
