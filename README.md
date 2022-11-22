# Mexico City public transit isochrones

This is the code for the [Mexico City public transit isochrones map](https://3d.bk.tudelft.nl/ken/maps/mexico-city/). The concept and a few styling ideas are based on [Chronotrains](https://www.chronotrains.com/). The data comes from the [Mexico city static GTFS](https://datos.cdmx.gob.mx/dataset/gtfs).

## How are the isochrones computed?

The isochrones are computed based on a network on top of an [H3](https://h3geo.org) hexagon grid with edges derived from GTFS trips (worst case wait time + travel time) and additional edges between adjacent hexagons (based on walking speed). The map uses the trips available based on a mid-day weekday schedule (Wednesday at 12:35).

## How to build the web map?

- `npm install`
- `npm start`

## Required libraries

- [H3](https://h3geo.org)
- [GDAL](https://gdal.org/)
- Niels Lohmann's [JSON for Modern C++](https://github.com/nlohmann/json)

## Known limitations

- We assume that it is possible to walk at the same speed between any two adjacent hexagons (even when there's no direct road connection)
- Issues in the Mexico City GTFS (eg Metro lines 1 and 12 in normal operation)
- GDAL is not robust when dissolving polygons (some missing isochrones)