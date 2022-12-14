import maplibre, { GeoJSONSource, MapMouseEvent } from 'maplibre-gl';

const base_url = "http://localhost:9080/example/dev-bundle";

const map = new maplibre.Map( {
    container: 'map',
    style: 'https://api.maptiler.com/maps/47a11a62-e38e-4021-a133-9aecd60c679a/style.json?key=wyRKCoUfzSqmjQWaOFIp',
    center: [ - 99.12, 19.43 ], // starting position [lng, lat]
    zoom: 12 // starting zoom
} );

// window.map = map;

map.on('load', () => {
    map.addSource("hovered_station", {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
    });
    map.addLayer({
        "id": "hovered_station_text",
        "type": "symbol",
        "source": "hovered_station",
        "layout": {
            "text-field": ["get", "name"],
            "text-anchor": "left",
            "text-offset": [0.5, 0],
            "text-size": [
                "interpolate",
                ["linear"],
                ["zoom"],
                13,
                14,
                15,
                20],
            "text-font": ["Noto Sans Bold"]
        },
        "paint": {
            "text-halo-width": 1,
            "text-halo-color": "#ffffff"
        },
    },
    "place_label_continent");
    map.addLayer({
        "id": "hovered_station",
        "type": "circle",
        "source": "hovered_station",
        "paint": {
            "circle-radius": [
                "interpolate",
                ["linear"],
                ["zoom"],
                13,
                3,
                15,
                6],
            "circle-color": "#000000",
            "circle-stroke-width": 1,
            "circle-stroke-color": "#ffffff"
        }
    },
    "hovered_station_text");
    

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
            ],
          },
        },
        "place_label_village"
    );
    map.addLayer(
    {
        id: "isochrones-outline",
        type: "line",
        source: "isochrones",
        layout: {},
        paint: {
            "line-color": [
                "interpolate",
                ["linear"],
                ["get", "duration"],
                0,
                "rgba(189,0,38,0.8)",
                0.25,
                "rgba(240,59,32,0.8)",
                0.5,
                "rgba(253,141,60,0.8)",
                0.75,
                "rgba(254,204,92,0.8)",
                1.0,
                "rgba(254,217,118,0.8)"
            ],
            "line-width": 1.5,
          },
        },
        "isochrones"
    );

    map.addSource("stations", {
        "type": "geojson",
        "data": `${base_url}/data/starting_points.geojson`
    });
    map.addLayer({
        "id": "starting_points",
        "type": "circle",
        "source": "stations",
        "paint": {
            "circle-radius": [
                "interpolate",
                ["linear"],
                ["zoom"],
                13,
                3,
                15,
                6],
            "circle-color": "#000000",
            "circle-stroke-width": 1,
            "circle-stroke-color": "#ffffff"
        }
    },
    "isochrones");
    map.addLayer({
        "id": "starting_points_text",
        "type": "symbol",
        "source": "stations",
        "layout": {
            "text-field": ["get", "name"],
            "text-anchor": "left",
            "text-offset": [0.5, 0],
            "text-size": [
                "interpolate",
                ["linear"],
                ["zoom"],
                13,
                10,
                15,
                20],
            "text-font": ["Noto Sans Bold"]
        },
        "paint": {
            "text-halo-width": 3,
            "text-halo-color": "#ffffff"
        },
        "minzoom": 13
    },
    "starting_points");

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
        (map.getSource("hovered_station") as GeoJSONSource).setData(station);
        const hoveredStation = station.properties?.id as Number;

        fetch(`${base_url}/data/isochrones/${hoveredStation}.geojson`).then(data => {
            data.json().then(json => {
                (map.getSource("isochrones") as GeoJSONSource).setData(json);
            });
        });
    } else {
        (map.getSource("hovered_station") as GeoJSONSource).setData({ type: "FeatureCollection", features: [] });
        (map.getSource("isochrones") as GeoJSONSource).setData({ type: "FeatureCollection", features: [] });
    }
}