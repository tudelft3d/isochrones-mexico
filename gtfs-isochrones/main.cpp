#include <iostream>
#include <fstream>
#include <sstream>

#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>

#include <ogrsf_frmts.h>
#include <h3/h3api.h>
#include <nlohmann/json.hpp>

struct Agency {
  std::string id, name;
};

struct Service {
  std::string id;
  bool monday, tuesday, wednesday, thursday, friday, saturday, sunday;
};

struct Route {
  std::string id, agency, short_name, long_name;
};

struct Stop {
  std::string id, name, zone;
  double lat, lon;
  bool wheelchair_boarding;
};

struct TripFrequency {
  int start, end, headway;
};

struct TripStop {
  int sequence, arrival, departure;
  std::string stop;
};

struct Trip {
  std::string id, route, service, head_sign, short_name;
  std::vector<TripFrequency> frequencies;
  std::map<int, TripStop> stops;
};

struct Connection {
  double wait_time, travel_time;
  std::string how;
  H3Index to;
};

struct Hex {
  std::string stop_name, transport_type;
  std::vector<std::string> stops;
  std::vector<Connection> connections;
};

std::pair<std::unordered_map<H3Index, double>, std::unordered_map<H3Index, Connection>> compute_times(std::unordered_map<H3Index, Hex> &hexes, H3Index start) {
  std::unordered_map<H3Index, double> time;
  std::unordered_map<H3Index, Connection> previous;
  std::set<std::pair<double, H3Index>> queue;
  std::unordered_map<H3Index, std::set<std::pair<double, H3Index>>::iterator> queue_positions;
  for (auto const &hex: hexes) {
    if (hex.first == start) time[hex.first] = 0.0;
    else time[hex.first] = std::numeric_limits<double>::max();
    previous[hex.first];
    auto ret = queue.insert(std::make_pair(time[hex.first], hex.first));
    queue_positions[hex.first] = ret.first;
  } while (!queue.empty()) {
    H3Index closest = queue.begin()->second;
    queue.erase(queue.begin());
    queue_positions.erase(closest);
    for (auto const &connection: hexes[closest].connections) {
      double alternative = time[closest] + connection.travel_time;
      double wait_time = 0.0;
      if (previous[closest].how != connection.how) wait_time += connection.wait_time;
      alternative += wait_time;
      if (alternative < time[connection.to]) {
        time[connection.to] = alternative;
        previous[connection.to].to = closest;
        previous[connection.to].how = connection.how;
        previous[connection.to].travel_time = connection.travel_time;
        previous[connection.to].wait_time = wait_time;
        queue.erase(queue_positions[connection.to]);
        auto ret = queue.insert(std::make_pair(alternative, connection.to));
        queue_positions[connection.to] = ret.first;
      }
    }
  }
  
  return std::make_pair(time, previous);
}

nlohmann::json compute_isochrones(std::unordered_map<H3Index, double> &all_times, std::vector<double> &isochrone_times) {
  nlohmann::json isochrones;
  isochrones["type"] = "FeatureCollection";
  isochrones["properties"] = nlohmann::json::object();
  isochrones["features"] = nlohmann::json::array();
  for (auto const &isochrone_time: isochrone_times) {
    std::vector<H3Index> smaller_hexes;
    for (auto const &hex: all_times) {
      if (hex.second < isochrone_time) smaller_hexes.push_back(hex.first);
    } if (smaller_hexes.empty()) continue;
    isochrones["features"].push_back(nlohmann::json::object());
    isochrones["features"].back()["type"] = "Feature";
    isochrones["features"].back()["geometry"] = nlohmann::json::object();
    isochrones["features"].back()["geometry"]["type"] = "MultiPolygon";
    isochrones["features"].back()["geometry"]["coordinates"] = nlohmann::json::array();
    isochrones["features"].back()["properties"] = nlohmann::json::object();
    isochrones["features"].back()["properties"]["duration"] = isochrone_time;
    OGRMultiPolygon multipolygon;
    for (auto const &hex: smaller_hexes) {
      CellBoundary cb;
      cellToBoundary(hex, &cb);
      OGRPolygon polygon;
      OGRLinearRing ring;
      for (int vertex = 0; vertex < cb.numVerts; ++vertex) {
        ring.addPoint(radsToDegs(cb.verts[vertex].lng), radsToDegs(cb.verts[vertex].lat));
      } ring.addPoint(radsToDegs(cb.verts[0].lng), radsToDegs(cb.verts[0].lat));
      polygon.addRing(&ring);
      multipolygon.addGeometry(&polygon);
    } OGRGeometry *hex_union = multipolygon.UnionCascaded();
    if (hex_union->getGeometryType() == wkbMultiPolygon) {
      OGRMultiPolygon *multipolygon = static_cast<OGRMultiPolygon *>(hex_union);
      for (int current_polygon = 0; current_polygon < multipolygon->getNumGeometries(); ++current_polygon) {
        OGRPolygon *polygon = multipolygon->getGeometryRef(current_polygon);
        isochrones["features"].back()["geometry"]["coordinates"].push_back(nlohmann::json::array());
        isochrones["features"].back()["geometry"]["coordinates"].back().push_back(nlohmann::json::array());
        for (int current_point = 0; current_point < polygon->getExteriorRing()->getNumPoints(); ++current_point) {
          isochrones["features"].back()["geometry"]["coordinates"].back().back().push_back({polygon->getExteriorRing()->getX(current_point),
                                                                                            polygon->getExteriorRing()->getY(current_point)});
        } for (int current_ring = 0; current_ring < polygon->getNumInteriorRings(); ++current_ring) {
          OGRLinearRing *ring = polygon->getInteriorRing(current_ring);
          isochrones["features"].back()["geometry"]["coordinates"].back().push_back(nlohmann::json::array());
          for (int current_point = 0; current_point < ring->getNumPoints(); ++current_point) {
            isochrones["features"].back()["geometry"]["coordinates"].back().back().push_back({ring->getX(current_point),
                                                                                              ring->getY(current_point)});
          }
        }
      }
    }
  } return isochrones;
}

int main(int argc, const char * argv[]) {
  
  std::string gtfs_folder = "/Users/ken/Library/Mobile Documents/com~apple~CloudDocs/Teaching/data/gtfs";
  std::string starting_points_file = "/Users/ken/Downloads/starting_points.geojson";
  std::string isochrones_folder = "/Users/ken/Downloads/isochrones";
  const int h3_resolution = 10;
  const int hex_buffer_size = 25;
  const double walking_speed = 5.0; // km/h
  double times[] = {1.0, 0.75, 0.5, 0.25};
  
  std::string stops_file = gtfs_folder + "/stops.txt";
  std::string routes_file = gtfs_folder + "/routes.txt";
  std::string agencies_file = gtfs_folder + "/agency.txt";
  std::string services_file = gtfs_folder + "/calendar.txt";
  std::string trips_file = gtfs_folder + "/trips.txt";
  std::string frequencies_file = gtfs_folder + "/frequencies.txt";
  std::string stop_times_file = gtfs_folder + "/stop_times.txt";
  std::unordered_map<std::string, Stop> stops;
  std::unordered_map<std::string, Route> routes;
  std::unordered_map<std::string, Agency> agencies;
  std::unordered_map<std::string, Service> services;
  std::unordered_map<std::string, Trip> trips;
  std::unordered_map<H3Index, Hex> hexes;
  std::vector<double> isochrone_times(times, times + sizeof(times)/sizeof(double));
  
  // Load data
  std::cout << "Loading data..." << std::endl;
  
  std::ifstream stops_stream(stops_file);
  std::string line;
  getline(stops_stream, line);
  while (getline(stops_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements[1].front() == '"' && line_elements[2].back() == '"') {
      line_elements[1].erase(0, 1);
      line_elements[2].pop_back();
      line_elements[1] = line_elements[1] + line_elements[2];
      line_elements.erase(line_elements.begin()+2);
    } if (line_elements.size() != 6) {
      std::cout << "Problems parsing stops" << std::endl;
      return 1;
    } stops[line_elements[0]].id = line_elements[0];
    stops[line_elements[0]].name = line_elements[1];
    stops[line_elements[0]].lat = std::stod(line_elements[2]);
    stops[line_elements[0]].lon = std::stod(line_elements[3]);
    stops[line_elements[0]].zone = line_elements[4];
    if (line_elements[5] == "1") stops[line_elements[0]].wheelchair_boarding = true;
    else stops[line_elements[0]].wheelchair_boarding = false;
  } stops_stream.close();
  std::cout << "\t" << stops.size() << " stops" << std::endl;
//  for (auto const &stop: stops) std::cout << "Stop[" << stop.second.id << "]: " << stop.second.name << " at " << stop.second.lat << ", " << stop.second.lon << std::endl;
  
  std::ifstream routes_stream(routes_file);
  getline(routes_stream, line);
  while (getline(routes_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 7) {
      std::cout << "Problems parsing routes" << std::endl;
      return 1;
    } routes[line_elements[0]].id = line_elements[0];
    routes[line_elements[0]].agency = line_elements[1];
    routes[line_elements[0]].short_name = line_elements[2];
    routes[line_elements[0]].long_name = line_elements[3];
  } routes_stream.close();
  std::cout << "\t" << routes.size() << " routes" << std::endl;
//  for (auto const &route: routes) std::cout << "Route[" << route.second.id << "]: " << route.second.agency << " " << route.second.short_name << " is " << route.second.long_name << std::endl;
  
  std::ifstream agencies_stream(agencies_file);
  getline(agencies_stream, line);
  while (getline(agencies_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 5) {
      std::cout << "Problems parsing agencies" << std::endl;
      return 1;
    } agencies[line_elements[0]].id = line_elements[0];
    agencies[line_elements[0]].name = line_elements[1];
  } agencies_stream.close();
  std::cout << "\t" << agencies.size() << " agencies" << std::endl;
//  for (auto const &agency: agencies) std::cout << "Agency[" << agency.second.id << "]: " << agency.second.name << std::endl;
  
  std::ifstream services_stream(services_file);
  getline(services_stream, line);
  while (getline(services_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 10) {
      std::cout << "Problems parsing services" << std::endl;
      return 1;
    } services[line_elements[0]].id = line_elements[0];
    if (line_elements[1] == "1") services[line_elements[0]].monday = true;
    else services[line_elements[0]].monday = false;
    if (line_elements[2] == "1") services[line_elements[0]].tuesday = true;
    else services[line_elements[0]].tuesday = false;
    if (line_elements[3] == "1") services[line_elements[0]].wednesday = true;
    else services[line_elements[0]].wednesday = false;
    if (line_elements[4] == "1") services[line_elements[0]].thursday = true;
    else services[line_elements[0]].thursday = false;
    if (line_elements[5] == "1") services[line_elements[0]].friday = true;
    else services[line_elements[0]].friday = false;
    if (line_elements[6] == "1") services[line_elements[0]].saturday = true;
    else services[line_elements[0]].saturday = false;
    if (line_elements[7] == "1") services[line_elements[0]].sunday = true;
    else services[line_elements[0]].sunday = false;
  } services_stream.close();
  std::cout << "\t" << services.size() << " services" << std::endl;
//  for (auto const &service: services) std::cout << "Service[" << service.second.id << "]: M = " << service.second.monday << " Tu = " << service.second.tuesday << " W = " << service.second.wednesday << " Th = " << service.second.thursday << " F = " << service.second.friday << " Sa = " << service.second.saturday << " Su = " << service.second.sunday << std::endl;
  
  std::ifstream trips_stream(trips_file);
  getline(trips_stream, line);
  while (getline(trips_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 7) {
      std::cout << "Problems parsing trips" << std::endl;
      return 1;
    } trips[line_elements[2]].id = line_elements[2];
    trips[line_elements[2]].route = line_elements[0];
    trips[line_elements[2]].service = line_elements[1];
    trips[line_elements[2]].head_sign = line_elements[3];
    trips[line_elements[2]].short_name = line_elements[4];
  } trips_stream.close();
  std::cout << "\t" << trips.size() << " trips" << std::endl;
//  for (auto const &trip: trips) std::cout << "Trip[" << trip.second.id << "] from route " << trip.second.route << " service " << trip.second.service << ": " << trip.second.head_sign << std::endl;
  
  std::ifstream frequencies_stream(frequencies_file);
  getline(frequencies_stream, line);
  while (getline(frequencies_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 5) {
      std::cout << "Problems parsing frequencies" << std::endl;
      return 1;
    } std::stringstream start_stream(line_elements[1]), end_stream(line_elements[2]);
    std::vector<std::string> start_elements, end_elements;
    while (getline(start_stream, element, ':')) start_elements.push_back(element);
    if (start_elements.size() != 3) {
      std::cout << "Problems parsing frequency start time" << std::endl;
      return 1;
    } unsigned int start = 3600*std::stoi(start_elements[0]) + 60*std::stoi(start_elements[1]) + std::stoi(start_elements[2]);
    while (getline(end_stream, element, ':')) end_elements.push_back(element);
    if (end_elements.size() != 3) {
      std::cout << "Problems parsing frequency end time" << std::endl;
      return 1;
    } unsigned int end = 3600*std::stoi(end_elements[0]) + 60*std::stoi(end_elements[1]) + std::stoi(end_elements[2]);
    trips[line_elements[0]].frequencies.emplace_back();
    trips[line_elements[0]].frequencies.back().start = start;
    trips[line_elements[0]].frequencies.back().end = end;
    trips[line_elements[0]].frequencies.back().headway = std::stoi(line_elements[3]);
  } frequencies_stream.close();
  int frequencies_parsed = 0;
  for (auto const &trip: trips) frequencies_parsed += trip.second.frequencies.size();
  std::cout << "\t" << frequencies_parsed << " frequencies" << std::endl;
//  for (auto const &trip: trips) {
//    std::cout << "Trip[" << trip.second.id << "] frequencies:" << std::endl;
//    for (auto const &frequency: trip.second.frequencies) {
//      std::string start_hours = std::to_string(frequency.start/3600);
//      std::string start_minutes = std::to_string((frequency.start%3600)/60);
//      if ((frequency.start%3600)/60 < 10) start_minutes.insert(0, "0");
//      std::string end_hours = std::to_string(frequency.end/3600);
//      std::string end_minutes = std::to_string((frequency.end%3600)/60);
//      if ((frequency.end%3600)/60 < 10) end_minutes.insert(0, "0");
//      std::cout << "\tfrom " << start_hours << ":" << start_minutes << " to " << end_hours << ":" << end_minutes << " is " << frequency.headway << " seconds" << std::endl;
//    }
//  }
  
  std::ifstream stop_times_stream(stop_times_file);
  getline(stop_times_stream, line);
  while (getline(stop_times_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 6) {
      std::cout << "Problems parsing stop times" << std::endl;
      return 1;
    } std::stringstream arrival_stream(line_elements[1]), departure_stream(line_elements[2]);
    std::vector<std::string> arrival_elements, departure_elements;
    while (getline(arrival_stream, element, ':')) arrival_elements.push_back(element);
    if (arrival_elements.size() != 3) {
      std::cout << "Problems parsing stop arrival time" << std::endl;
      return 1;
    } unsigned int arrival = 3600*std::stoi(arrival_elements[0]) + 60*std::stoi(arrival_elements[1]) + std::stoi(arrival_elements[2]);
    while (getline(departure_stream, element, ':')) departure_elements.push_back(element);
    if (departure_elements.size() != 3) {
      std::cout << "Problems parsing stop departure time" << std::endl;
      return 1;
    } unsigned int departure = 3600*std::stoi(departure_elements[0]) + 60*std::stoi(departure_elements[1]) + std::stoi(departure_elements[2]);
    unsigned int sequence = std::stoi(line_elements[4]);
    trips[line_elements[0]].stops[sequence].sequence = sequence;
    trips[line_elements[0]].stops[sequence].arrival = arrival;
    trips[line_elements[0]].stops[sequence].departure = departure;
    trips[line_elements[0]].stops[sequence].stop = line_elements[3];
  } int stop_times_parsed = 0;
  for (auto const &trip: trips) stop_times_parsed += trip.second.stops.size();
  std::cout << "\t" << stop_times_parsed << " stop times" << std::endl;
//  for (auto const &trip: trips) {
//    std::cout << "Trip[" << trip.second.id << "] stops:" << std::endl;
//    for (auto const &stop: trip.second.stops) {
//      std::string arrival_hours = std::to_string(stop.second.arrival/3600);
//      std::string arrival_minutes = std::to_string((stop.second.arrival%3600)/60);
//      if ((stop.second.arrival%3600)/60 < 10) arrival_minutes.insert(0, "0");
//      std::string arrival_seconds = std::to_string(stop.second.arrival%60);
//      if (stop.second.arrival%60 < 10) arrival_seconds.insert(0, "0");
//      std::string departure_hours = std::to_string(stop.second.departure/3600);
//      std::string departure_minutes = std::to_string((stop.second.departure%3600)/60);
//      if ((stop.second.departure%3600)/60 < 10) departure_minutes.insert(0, "0");
//      std::string departure_seconds = std::to_string(stop.second.departure%60);
//      if (stop.second.departure%60 < 10) departure_seconds.insert(0, "0");
//      std::cout << "\t" << stop.first << ": from " << arrival_hours << ":" << arrival_minutes << ":" << arrival_seconds << " to " << departure_hours << ":" << departure_minutes << ":" << departure_seconds << " at " << stop.second.stop << std::endl;
//    }
//  }
  
  // Generate hexes with stops
  std::cout << "Generating hexes for stops..." << std::endl;
  for (auto const &stop: stops) {
    LatLng ll;
    ll.lat = degsToRads(stop.second.lat);
    ll.lng = degsToRads(stop.second.lon);
    H3Index hex;
    latLngToCell(&ll, h3_resolution, &hex);
    hexes[hex].stops.push_back(stop.first);
  }
  
  // Assign names to some hexes
  for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "METRO") continue;
    for (auto const &stop: trip.second.stops) {
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(3, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, 3, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = routes[trip.second.route].agency;
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  } for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "TL") continue;
    for (auto const &stop: trip.second.stops) {
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(3, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, 3, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = routes[trip.second.route].agency;
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  } for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "SUB") continue;
    for (auto const &stop: trip.second.stops) {
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(3, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, 3, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = routes[trip.second.route].agency;
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  } for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "MB") continue;
    if (routes[trip.second.route].short_name == "SE L12") continue;
    for (auto const &stop: trip.second.stops) {
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(3, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, 3, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty() && !stops[stop.second.stop].name.starts_with("Metro")) {
        hexes[hex].transport_type = routes[trip.second.route].agency;
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  } for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "CBB") continue;
    for (auto const &stop: trip.second.stops) {
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(3, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, 3, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = routes[trip.second.route].agency;
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  } for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "TROLE") continue;
    if (routes[trip.second.route].short_name != "10") continue;
    for (auto const &stop: trip.second.stops) {
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(3, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, 3, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = routes[trip.second.route].agency;
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  }
  
  // Add hexes near stops
  std::cout << "Adding hexes near stops..." << std::endl;
  std::unordered_set<H3Index> new_hexes;
  for (auto const &hex: hexes) {
    int64_t max_hexes;
    maxGridDiskSize(hex_buffer_size, &max_hexes);
    H3Index *hexes_within_distance = new H3Index[max_hexes];
    gridDisk(hex.first, hex_buffer_size, hexes_within_distance);
    for (int i = 0; i < max_hexes; ++i) {
      if (hexes_within_distance[i] != 0) new_hexes.insert(hexes_within_distance[i]);
    } delete []hexes_within_distance;
  } for (auto const &hex: new_hexes) hexes[hex];
  
  // Write starting points
  std::cout << "Writing isochrone starting points..." << std::endl;
  std::ofstream output_stream;
  output_stream.open(starting_points_file);
  nlohmann::json starting_points;
  starting_points["type"] = "FeatureCollection";
  starting_points["features"] = nlohmann::json::array();
  for (auto const &hex: hexes) {
    if (hex.second.stop_name.empty()) continue;
    LatLng ll;
    cellToLatLng(hex.first, &ll);
    starting_points["features"].push_back(nlohmann::json::object());
    starting_points["features"].back()["type"] = "Feature";
    starting_points["features"].back()["geometry"] = nlohmann::json::object();
    starting_points["features"].back()["geometry"]["type"] = "Point";
    starting_points["features"].back()["geometry"]["coordinates"] = {radsToDegs(ll.lng), radsToDegs(ll.lat)};
    starting_points["features"].back()["properties"] = nlohmann::json::object();
    starting_points["features"].back()["properties"]["id"] = std::to_string(hex.first);
    starting_points["features"].back()["properties"]["system"] = hex.second.transport_type;
    starting_points["features"].back()["properties"]["name"] = hex.second.stop_name;
  } output_stream << starting_points.dump() << std::endl;
  output_stream.close();
  
  return 0;
  
  // Add walking connections
  std::cout << "Adding walking connections..." << std::endl;
  for (auto &hex: hexes) {
    int64_t max_hexes;
    maxGridDiskSize(1, &max_hexes);
    H3Index *hexes_within_distance = new H3Index[max_hexes];
    gridDisk(hex.first, 1, hexes_within_distance);
    LatLng from_ll;
    cellToLatLng(hex.first, &from_ll);
    for (int i = 0; i < max_hexes; ++i) {
      if (hexes_within_distance[i] != 0 && hexes.count(hexes_within_distance[i])) {
        LatLng to_ll;
        cellToLatLng(hexes_within_distance[i], &to_ll);
        double distance = greatCircleDistanceKm(&from_ll, &to_ll);
        hex.second.connections.emplace_back();
        hex.second.connections.back().to = hexes_within_distance[i];
        hex.second.connections.back().travel_time = distance/walking_speed;
        hex.second.connections.back().wait_time = 0.0;
        hex.second.connections.back().how = "walk";
      }
    }
  }
  
  // Add transit connections
  for (auto const &trip: trips) {
    
    // Filtering
    if (!services[trip.second.service].wednesday) continue;
    const int check_time = 3600*12+60*35;
    int headway = std::numeric_limits<int>::max();
    for (auto const &frequency: trip.second.frequencies) {
      if (frequency.start <= check_time && frequency.end >= check_time) {
        if (frequency.headway < headway) headway = frequency.headway;
      }
    } if (headway == std::numeric_limits<int>::max()) continue;
    
    // Add connection
//    std::cout << "Trip[" << trip.second.id << "] " << trip.second.short_name << " with headway of " << headway << " seconds:" << std::endl;
    std::map<int, TripStop>::const_iterator previous_stop = trip.second.stops.begin(), next_stop = trip.second.stops.begin();
    ++next_stop;
    while (next_stop != trip.second.stops.end()) {
      int travel_time = next_stop->second.departure-previous_stop->second.departure;
//      std::cout << "\t" << previous_stop->second.stop << " to " << next_stop->second.stop << " in " << travel_time << " seconds" << std::endl;
      LatLng previous_ll, next_ll;
      H3Index previous_hex, next_hex;
      previous_ll.lat = degsToRads(stops[previous_stop->second.stop].lat);
      previous_ll.lng = degsToRads(stops[previous_stop->second.stop].lon);
      latLngToCell(&previous_ll, h3_resolution, &previous_hex);
      next_ll.lat = degsToRads(stops[next_stop->second.stop].lat);
      next_ll.lng = degsToRads(stops[next_stop->second.stop].lon);
      latLngToCell(&next_ll, h3_resolution, &next_hex);
      hexes[previous_hex].connections.emplace_back();
      hexes[previous_hex].connections.back().to = next_hex;
      hexes[previous_hex].connections.back().travel_time = travel_time/3600.0;
      hexes[previous_hex].connections.back().wait_time = headway/3600.0;
      hexes[previous_hex].connections.back().how = trip.second.short_name;
      ++previous_stop;
      ++next_stop;
    }
  }
  
  // Compute isochrones
  for (auto const &hex: hexes) {
    if (hex.second.stop_name.empty()) continue;
    std::cout << "Computing isochrone for " << hex.second.transport_type << " " << hex.second.stop_name << "..." << std::endl;
    auto time_and_previous = compute_times(hexes, hex.first);
    
    nlohmann::json isochrones = compute_isochrones(time_and_previous.first, isochrone_times);
    isochrones["properties"]["id"] = std::to_string(hex.first);
    isochrones["properties"]["system"] = hex.second.transport_type;
    isochrones["properties"]["name"] = hex.second.stop_name;
    output_stream.open(isochrones_folder + "/" + std::to_string(hex.first) + ".geojson");
    output_stream << isochrones.dump() << std::endl;
    output_stream.close();
  }
  
  // Write hexes
//  std::cout << "Writing hexes..." << std::endl;
//  GDALAllRegister();
//  const char *driver_name = "GPKG";
//  const char *output_file = "/Users/ken/Downloads/hexes.gpkg";
//  GDALDriver *driver = GetGDALDriverManager()->GetDriverByName(driver_name);
//  GDALDataset *output_dataset = (GDALDataset*) GDALOpenEx(output_file, GDAL_OF_READONLY, NULL, NULL, NULL);
//  if (output_dataset != NULL) {
//    std::cout << "Overwriting file..." << std::endl;
//    if (driver->Delete(output_file)!= CE_None) {
//      std::cerr << "Error: Couldn't erase existing file to create output file" << std::endl;
//      return 1;
//    } GDALClose(output_dataset);
//  } std::cout << "Writing output file... " << std::endl;
//  output_dataset = driver->Create(output_file,0,0,0,GDT_Unknown,NULL);
//  if (output_dataset == NULL) {
//    std::cout << "Error: Could not create output file." << std::endl;
//    return 1;
//  } OGRSpatialReference spatial_reference;
//  spatial_reference.importFromEPSG(4326);
  
//  OGRLayer *hexes_layer = output_dataset->CreateLayer("hexes", &spatial_reference, wkbPolygon, NULL);
//  if (hexes_layer == NULL) {
//    std::cerr << "Error: Could not create hexes layer." << std::endl;
//    return 1;
//  } OGRFieldDefn time_field("time", OFTReal);
//  if (hexes_layer->CreateField(&time_field) != OGRERR_NONE) {
//    std::cout << "Error: Could not create field time." << std::endl;
//    return 1;
//  } for (auto const &hex: hexes) {
//    CellBoundary cb;
//    cellToBoundary(hex.first, &cb);
//    OGRFeature *hex_feature = OGRFeature::CreateFeature(hexes_layer->GetLayerDefn());
//    hex_feature->SetField("time", time_and_previous.first[hex.first]);
//    OGRPolygon hex_polygon;
//    OGRLinearRing hex_ring;
//    for (int vertex = 0; vertex < cb.numVerts; ++vertex) hex_ring.addPoint(radsToDegs(cb.verts[vertex].lng),
//                                                                           radsToDegs(cb.verts[vertex].lat));
//    hex_ring.addPoint(radsToDegs(cb.verts[0].lng), radsToDegs(cb.verts[0].lat));
//    hex_polygon.addRing(&hex_ring);
//    hex_feature->SetGeometry(&hex_polygon);
//    if (hexes_layer->CreateFeature(hex_feature) != OGRERR_NONE) {
//      std::cerr << "Error: Could not create hex feature." << std::endl;
//      return 1;
//    } OGRFeature::DestroyFeature(hex_feature);
//  }
  
//  OGRLayer *connections_layer = output_dataset->CreateLayer("connections", &spatial_reference, wkbLineString, NULL);
//  if (connections_layer == NULL) {
//    std::cerr << "Error: Could not create connections layer." << std::endl;
//    return 1;
//  } OGRFieldDefn wait_time_field("wait_time", OFTReal);
//  if (connections_layer->CreateField(&wait_time_field) != OGRERR_NONE) {
//    std::cout << "Error: Could not create field wait_time." << std::endl;
//    return 1;
//  } OGRFieldDefn travel_time_field("travel_time", OFTReal);
//  if (connections_layer->CreateField(&travel_time_field) != OGRERR_NONE) {
//    std::cout << "Error: Could not create field travel_time." << std::endl;
//    return 1;
//  }
//  for (auto const &hex: hexes) {
//    for (auto const &connection: hex.second.connections) {
//      OGRFeature *connection_feature = OGRFeature::CreateFeature(connections_layer->GetLayerDefn());
//      connection_feature->SetField("wait_time", connection.wait_time);
//      connection_feature->SetField("travel_time", connection.travel_time);
//      OGRLineString connection_line_string;
//      LatLng hex_ll, connection_ll;
//      cellToLatLng(hex.first, &hex_ll);
//      cellToLatLng(connection.to, &connection_ll);
//      connection_line_string.addPoint(radsToDegs(hex_ll.lng), radsToDegs(hex_ll.lat));
//      connection_line_string.addPoint(radsToDegs(connection_ll.lng), radsToDegs(connection_ll.lat));
//      connection_feature->SetGeometry(&connection_line_string);
//      if (connections_layer->CreateFeature(connection_feature) != OGRERR_NONE) {
//        std::cerr << "Error: Could not create connection feature." << std::endl;
//        return 1;
//      } OGRFeature::DestroyFeature(connection_feature);
//    }
//  }
  
//  GDALClose(output_dataset);
  
  return 0;
}