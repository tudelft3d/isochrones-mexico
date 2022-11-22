# Mexico City public transit isochrones

This is the code for the [Mexico City public transit isochrones map](https://3d.bk.tudelft.nl/ken/maps/mexico-city/).

The base map is the Toner style from [Maptiler](https://www.maptiler.com/), itself based on the old style from [Stamen Design](http://maps.stamen.com/toner/). The  concept and a few styling code excerpts are based on [Chronotrains](https://www.chronotrains.com/). The public transit data comes from the [Mexico city static GTFS](https://datos.cdmx.gob.mx/dataset/gtfs).

## How are the isochrones computed?

The C++ code for this is in the gtfs-isochrones folder.

The isochrones are computed based on a network on top of an [H3](https://h3geo.org) hexagon grid with edges derived from GTFS trips (worst case wait time + travel time) and additional edges between adjacent hexagons (based on walking speed). The map uses the trips available based on a mid-day weekday schedule (Wednesday at 12:35).

## How to build the web map?

- `npm install`
- `npm start`

## Required libraries

- [H3](https://h3geo.org)
- [GDAL](https://gdal.org/)
- Niels Lohmann's [JSON for Modern C++](https://github.com/nlohmann/json)

## Known limitations

- We assume that it is possible to walk at the same speed between any two adjacent H3 hexagons (even when there's no direct road connection or the route is not straight)
- Issues in the Mexico City GTFS (eg Metro lines 1 and 12 in normal operation)
- Limited coverage of the Mexico City GTFS (eg only a few private operators, limited coverage outside of Mexico City proper)
- GDAL is not always robust when dissolving polygons (see some missing isochrones)