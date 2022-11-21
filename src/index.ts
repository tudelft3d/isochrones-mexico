import maplibre, { GeoJSONSource, MapMouseEvent } from 'maplibre-gl';

const base_url = "http://localhost:9080/example/dev-bundle";

const map = new maplibre.Map( {
    container: 'map',
    style: 'https://api.maptiler.com/maps/basic/style.json?key=get_your_own_OpIi9ZULNHzrESv6T2vL', //'https://demotiles.maplibre.org/style.json', // stylesheet location
    center: [ - 99.12, 19.43 ], // starting position [lng, lat]
    zoom: 12 // starting zoom
} );

// window.map = map;

map.on('load', () => {
    map.addSource("stations", {
        "type": "geojson",
        "data": `${base_url}/data/starting_points.geojson`
    });
    map.addLayer({
        "id": "starting_points",
        "type": "circle",
        "source": "stations",
        "paint": {
            "circle-radius": 6,
            "circle-color": "#000000"
        }
    },
    "label_airport");

    map.addSource("isochrones", {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
    });

    map.addLayer(
    {
        id: "isochrones",
        type: "fill",
        source: "isochrones",
        layout: {},
        paint: {
            "fill-opacity": [
              "interpolate",
              ["linear"],
              ["zoom"],
              7,
              0.7,
              15,
              0.2,
            ],
            "fill-color": [
              "interpolate",
              ["linear"],
              ["get", "duration"],
              0,
              "rgba(189,0,38,0.9)",
              0.25,
              "rgba(240,59,32,0.8)",
              0.5,
              "rgba(253,141,60,0.7)",
              0.75,
              "rgba(254,204,92,0.6)",
              1,
              "rgba(254,217,118, 0.5)"
            //   300,
            //   "rgba(255,255,178, 0.4)",
            ],
          },
        },
        "starting_points"
    );

    map.on('mousemove', onMouseMove);
});

function onMouseMove(e: MapMouseEvent) {
    const features = map.queryRenderedFeatures(
        [
            [e.point.x - 10, e.point.y - 10],
            [e.point.x + 10, e.point.y + 10],
        ],
        {
            layers: ["starting_points"],
        }
    );

    if (features.length) {
        const station = features[features.length - 1]; // the largest according to the API scoring
        const hoveredStation = station.properties?.new_id as Number;

        fetch(`${base_url}/data/isochrones/${hoveredStation}.geojson`).then(data => {
            data.json().then(json => {
                (map.getSource("isochrones") as GeoJSONSource).setData(json);
            });
        });
    } else {
        (map.getSource("isochrones") as GeoJSONSource).setData({ type: "FeatureCollection", features: [] });
    }
}