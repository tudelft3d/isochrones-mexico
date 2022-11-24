#include "Isochrone_generator.hpp"

int main(int argc, const char * argv[]) {
  
  std::string gtfs_folder = "/Users/ken/Library/Mobile Documents/com~apple~CloudDocs/Teaching/data/gtfs";
  std::string starting_points_file = "/Users/ken/Downloads/starting_points.geojson";
  std::string isochrones_folder = "/Users/ken/Downloads/isochrones";
  const int h3_resolution = 10;
  const int hex_buffer_size = 50; // buffer around stops to cover entire region
  const double walking_speed = 5.0; // km/h
  double times[] = {1.0, 0.75, 0.5, 0.25}; // large to small
  std::vector<double> isochrone_times(times, times + sizeof(times)/sizeof(double));
  
  Isochrone_generator isochrone_generator(h3_resolution);
  
  // Load GTFS data
  int error = isochrone_generator.load_gtfs_data(gtfs_folder);
  if (error) return error;
  
  // Generate hexes for region (within buffer of stops)
  isochrone_generator.create_hexes_for_stops(hex_buffer_size);
  
  // Define starting points (hardcoded with interesting stope for Mexico City)
  isochrone_generator.create_mexico_city_starting_points();
  
  // Write starting points
  isochrone_generator.write_starting_points(starting_points_file);
  
  // Add walking connections between adjacent hexagons
  isochrone_generator.add_walking_connections(walking_speed);
  
  // Add transit connections
  isochrone_generator.add_transit_connections();
  
  // Compute isochrones
  isochrone_generator.write_isochrones_for_starting_points(isochrones_folder, isochrone_times);
  
  // Small test
//  LatLng ll;
//  H3Index hex;
//  ll.lat = degsToRads(19.430745);
//  ll.lng = degsToRads(-99.053286);
//  latLngToCell(&ll, h3_resolution, &hex);
//  auto time_and_previous = isochrone_generator.compute_routes_from_hex(hex);
//  std::string hexes_geojson = "/Users/ken/Downloads/hexes.geojson";
//  isochrone_generator.write_hexes_geojson(hexes_geojson, time_and_previous);

  // Write hexes and connections (for debugging)
//  std::string hexes_gpkg = "/Users/ken/Downloads/hexes.gpkg";
//  std::string connections_gpkg = "/Users/ken/Downloads/connections.gpkg";
//  error = isochrone_generator.write_hexes_gpkg(hexes_gpkg);
//  if (error) return 1;
//  error = isochrone_generator.write_connections_gpkg(connections_gpkg);
//  if (error) return 1;
//  
//  // GeoJSON alternative (quick to write but slow to open)
//  std::string hexes_geojson = "/Users/ken/Downloads/hexes.geojson";
//  std::string connections_geojson = "/Users/ken/Downloads/connections.geojson";
//  isochrone_generator.write_hexes_geojson(hexes_geojson);
//  isochrone_generator.write_connections_geojson(connections_geojson);
  
  return 0;
}
