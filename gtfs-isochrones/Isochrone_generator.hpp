#ifndef Isochrone_generator_hpp
#define Isochrone_generator_hpp

#include <iostream>
#include <fstream>
#include <sstream>
#include <mach/mach.h>

#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>

#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <ogrsf_frmts.h>
#include <h3/h3api.h>
#include <nlohmann/json.hpp>

#include "Data_model.h"

class RoadHandler : public osmium::handler::Handler {
  LatLng ll;
  H3Index hex_prev, hex_next;
public:
  std::unordered_map<H3Index, Hex> *hexes;
  int h3_resolution;
  
  std::unordered_map<H3Index, std::unordered_set<H3Index>> walking_connections;
  
  void way(const osmium::Way& way) {
    const char* highway = way.tags()["highway"];
    if (highway && way.nodes().size() > 1) {
      osmium::NodeRefList::const_iterator next = way.nodes().begin();
      ll.lat = degsToRads(next->lat());
      ll.lng = degsToRads(next->lon());
      latLngToCell(&ll, h3_resolution, &hex_next);
      ++next;
      while (next != way.nodes().end()) {
        hex_prev = hex_next;
        ll.lat = degsToRads(next->lat());
        ll.lng = degsToRads(next->lon());
        latLngToCell(&ll, h3_resolution, &hex_next);
        
        if (hexes->count(hex_prev) && hexes->count(hex_prev)) {
          walking_connections[hex_prev].insert(hex_next);
          walking_connections[hex_next].insert(hex_prev);
        }
        
        ++next;
      }
    }
  }
};

class Isochrone_generator {
  int h3_resolution;
  
  std::unordered_map<std::string, Stop> stops;
  std::unordered_map<std::string, Route> routes;
  std::unordered_map<std::string, Agency> agencies;
  std::unordered_map<std::string, Service> services;
  std::unordered_map<std::string, Trip> trips;
  std::unordered_map<H3Index, Hex> hexes;
  
  void print_timer(clock_t start_time);
  std::map<std::string, std::size_t> read_header(std::string &header_line);
  void add_walking_connection(H3Index start, H3Index end, double walking_speed);
  std::pair<std::unordered_map<H3Index, double>, std::unordered_map<H3Index, Connection>> compute_routes_from_hex(H3Index start);
  nlohmann::json create_isochrones_from_routes(std::unordered_map<H3Index, double> &all_times, std::vector<double> &isochrone_times);
public:
  Isochrone_generator(int h3_resolution);
  
  int load_gtfs_data(std::string &gtfs_folder);
  void create_hexes_for_stops(int hex_buffer_size);
  void create_mexico_city_starting_points();
  void write_starting_points(std::string &starting_points_file);
  void add_walking_connections(std::string &osm, double walking_speed);
  void add_transit_connections();
  void write_isochrones_for_starting_points(std::string &isochrones_folder, std::vector<double> &isochrone_times);
  int write_hexes_gpkg(std::string &hexes_file);
  int write_connections_gpkg(std::string &connections_file);
  void write_hexes_geojson(std::string &hexes_file);
  void write_connections_geojson(std::string &connections_file);
};

#endif /* Isochrone_generator_hpp */
