# Isochrone generator: performance and output-size analysis

Status: the main wins are implemented. GTFS parses cleanly (11362 stops, 301
routes, 1205 trips, 42789 stop times) and a full run for 436 starting points
(431 original + 5 Interurbano/Tren El Insurgente stations) takes ~4 minutes.
Per-point isochrone time is ~0.5 s. Two costs remain: the OSM
walking-connections phase (~30 s, once per run) and the per-point loop
(~3.4 min, serial — now parallelized across cores).

## Completed

### 1. GEOS union replaced with H3 `cellsToLinkedMultiPolygon`

`create_isochrones_from_routes` (`Isochrone_generator.cpp:189`) previously built
an `OGRMultiPolygon` from one `OGRPolygon` per hex and called `UnionCascaded()`
(GEOS). It now collects the H3 indices per time threshold and calls
`cellsToLinkedMultiPolygon` (`Isochrone_generator.cpp:204`), which traces the
exact shared-edge boundary in **O(boundary cells)**, no GEOS, no floating-point
union. Per-point write times dropped from a 2.6–31 s range (median ~4.3 s) to
~0.5 s max.

Holes are preserved: within a `LinkedGeoPolygon` the first loop is the exterior
boundary and subsequent loops are holes (`LinkedGeoPolygon::next` chains the
disjoint polygons of a MultiPolygon). The emitter closes each ring (H3 returns
unclosed loops), converts radians→degrees and swaps `[lat,lng]`→`[lng,lat]`, and
rounds to 6 decimals. GDAL is no longer needed for this step (it remains only
for the debug gpkg writers).

### 2. Coordinate rounding

Coordinates are rounded to 6 decimals (~0.1 m), far more precision than a
res-10 hex grid needs. Measured on real output: 331 KB → 192 KB per file.
Current total output is ~63 MB / 436 files (~144 KB avg).

### 3. Dijkstra early stop

`compute_routes_from_hex(start, max_time)` (`Isochrone_generator.cpp:154`) takes
the largest isochrone threshold and `break`s as soon as the queue's smallest
distance reaches it (`Isochrone_generator.cpp:166`). All edge weights are
non-negative and every isochrone filters on `< threshold` with max 1.0 h, so
skipped nodes can never enter an isochrone. This removed the per-point outliers
(median ~0.4 s, max ~0.51 s).

## Remaining bottlenecks

### OSM walking connections (`add_walking_connections`, `Isochrone_generator.cpp:913`)

~30 s per run: parsing the country-wide Mexico PBF with libosmium plus the
"remove thin holes" logic. This is now the largest single phase besides the
per-point loop. Ideas: cache the walking connections so they only need to be
computed once per data set, or read only the region of the PBF around the stops.

### The per-point loop (`write_isochrones_for_starting_points`)

436 starting points × ~0.5 s ≈ 3.4 min, serial. The starting points are
independent — parallelize with `std::thread`/`dispatch_apply` for a core-count
speedup (8× on an M-series chip). Now parallel: the starting points are
collected into a vector, distributed to
`std::thread::hardware_concurrency()` workers via an atomic index, and each
worker runs `compute_routes_from_hex` + `create_isochrones_from_routes` on its
own data and writes its own file. Output is mutex-guarded. `compute_routes_from_hex`
only reads the shared `hexes` map (`.at()` instead of `operator[]`). Expect
roughly a core-count speedup on an M-series chip.

### Minor: 4× re-scan in `create_isochrones_from_routes`

The time thresholds are nested but each one re-scans all hexes. This is cheap
relative to the Dijkstra and only worth fixing if the loop is parallelized.

## Remaining recommendations for output size

- **Douglas-Peucker simplification** on each ring after merging (GDAL
  `SimplifyPreserveTopology`, tolerance ~1e-5 °). Long straight runs along the
  hex grid still keep a vertex per hex corner (rings up to ~2300 vertices).
  Combined with rounding this likely gets files down to roughly a quarter of
  current size.
- Further out: TopoJSON would dedupe the shared edges between the 4 nested
  isochrone rings, and the web map could fetch gzipped files.

## Suggested implementation order

1. [x] `cellsToLinkedMultiPolygon` + coordinate rounding.
2. [x] Dijkstra early-stop.
3. [x] Parallelism across starting points.
4. [ ] Douglas-Peucker simplification.
