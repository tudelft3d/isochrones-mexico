# Mexico City public transit isochrones

This is the code for the [Mexico City public transit isochrones map](https://3d.bk.tudelft.nl/ken/maps/mexico-city/).

The map is built with [MapLibre](https://maplibre.org/) using vector tiles and the Toner style from [Maptiler](https://www.maptiler.com/), itself based on the old style from [Stamen Design](http://maps.stamen.com/toner/). Maptiler uses data from [OpenStreetMap](https://www.openstreetmap.org) to generate the vector tiles. The concept and some bits of code are taken from [Chronotrains](https://www.chronotrains.com/). The public transit data comes from November 11, 2022 update of the [Mexico city static GTFS](https://datos.cdmx.gob.mx/dataset/gtfs).

## How are the isochrones computed?

The C++ code for this is in the gtfs-isochrones folder.

The isochrones, which you can download [here](https://3d.bk.tudelft.nl/ken/maps/mexico-city/data/data.zip), are computed based on a network on top of an [H3](https://h3geo.org) hexagon grid with edges derived from GTFS trips (worst case wait time + travel time) and additional edges between adjacent hexagons (based on walking speed). The map uses the trips available based on a mid-day weekday schedule (Wednesday at 12:35).

## How to build the web map?

- `npm install`
- `npm start`

## Required libraries

- [H3](https://h3geo.org)
- [GDAL](https://gdal.org/)
- Niels Lohmann's [JSON for Modern C++](https://github.com/nlohmann/json)

## Known limitations

- We assume that it is possible to walk at the same speed between any two adjacent H3 hexagons (even when there's no direct road connection, the route is not straight or the terrain isn't flat)
- Issues in the Mexico City GTFS (eg Metro lines 1 and 12 operating normally, some inconsistent station locations)
- Limited coverage of the Mexico City GTFS (eg only a few private operators, limited coverage outside of Mexico City proper)
