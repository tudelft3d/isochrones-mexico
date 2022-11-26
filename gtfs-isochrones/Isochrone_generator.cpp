#include "Isochrone_generator.hpp"

void Isochrone_generator::print_timer(clock_t start_time) {
  clock_t stop_time = clock();
  double seconds = (stop_time-start_time)/(double)CLOCKS_PER_SEC;
  std::cout << seconds << " seconds";
}

std::map<std::string, std::size_t> Isochrone_generator::read_header(std::string &header_line) {
  std::map<std::string, std::size_t> header_fields;
  std::stringstream line_stream(header_line);
  std::vector<std::string> line_elements;
  std::string element;
  while (getline(line_stream, element, ',')) {
    while (element.back() == '\r' || element.back() == '\n') element.pop_back();
//    std::cout << "\"" << element << "\"" << std::endl;
    header_fields[element] = header_fields.size();
  } return header_fields;
}

void Isochrone_generator::add_walking_connection(H3Index start, H3Index end, double walking_speed) {
  if (start == end) return;
  int64_t distance_cells;
  gridDistance(start, end, &distance_cells);
  LatLng start_ll, end_ll;
  cellToLatLng(start, &start_ll);
  cellToLatLng(end, &end_ll);
  if (distance_cells == 1) {
    double distance_km = greatCircleDistanceKm(&start_ll, &end_ll);
    hexes[start].connections.emplace_back();
    hexes[start].connections.back().to = end;
    hexes[start].connections.back().travel_time = distance_km/walking_speed;
    hexes[start].connections.back().wait_time = 0.0;
    hexes[start].connections.back().how = "walk";
  } else {
    LatLng mid_ll;
    mid_ll.lat = (start_ll.lat+end_ll.lat)/2.0;
    mid_ll.lng = (start_ll.lng+end_ll.lng)/2.0;
    H3Index mid;
    latLngToCell(&mid_ll, h3_resolution, &mid);
    add_walking_connection(start, mid, walking_speed);
    add_walking_connection(mid, end, walking_speed);
  }
}

std::pair<std::unordered_map<H3Index, double>, std::unordered_map<H3Index, Connection>> Isochrone_generator::compute_routes_from_hex(H3Index start) {
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
  } return std::make_pair(time, previous);
}

nlohmann::json Isochrone_generator::create_isochrones_from_routes(std::unordered_map<H3Index, double> &all_times, std::vector<double> &isochrone_times) {
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
      isochrones["features"].back()["geometry"]["type"] = "MultiPolygon";
      isochrones["features"].back()["geometry"]["coordinates"] = nlohmann::json::array();
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
    } else if (hex_union->getGeometryType() == wkbPolygon) {
      OGRPolygon *polygon = static_cast<OGRPolygon *>(hex_union);
      isochrones["features"].back()["geometry"]["type"] = "Polygon";
      isochrones["features"].back()["geometry"]["coordinates"] = nlohmann::json::array();
      isochrones["features"].back()["geometry"]["coordinates"].push_back(nlohmann::json::array());
      for (int current_point = 0; current_point < polygon->getExteriorRing()->getNumPoints(); ++current_point) {
        isochrones["features"].back()["geometry"]["coordinates"].back().push_back({polygon->getExteriorRing()->getX(current_point),
                                                                                   polygon->getExteriorRing()->getY(current_point)});
      } for (int current_ring = 0; current_ring < polygon->getNumInteriorRings(); ++current_ring) {
        OGRLinearRing *ring = polygon->getInteriorRing(current_ring);
        isochrones["features"].back()["geometry"]["coordinates"].push_back(nlohmann::json::array());
        for (int current_point = 0; current_point < ring->getNumPoints(); ++current_point) {
          isochrones["features"].back()["geometry"]["coordinates"].back().push_back({ring->getX(current_point),
                                                                                     ring->getY(current_point)});
        }
      }
    } else {
      std::cout << "\terror: union produced unknown output type" << std::endl;
    }
  }
  
  return isochrones;
}

Isochrone_generator::Isochrone_generator(int h3_resolution) {
  this->h3_resolution = h3_resolution;
}

int Isochrone_generator::load_gtfs_data(std::string &gtfs_folder) {
  std::cout << "Loading data..." << std::endl;
  clock_t start_time = clock();
  
  std::string stops_file = gtfs_folder + "/stops.txt";
  std::string routes_file = gtfs_folder + "/routes.txt";
  std::string agencies_file = gtfs_folder + "/agency.txt";
  std::string services_file = gtfs_folder + "/calendar.txt";
  std::string trips_file = gtfs_folder + "/trips.txt";
  std::string frequencies_file = gtfs_folder + "/frequencies.txt";
  std::string stop_times_file = gtfs_folder + "/stop_times.txt";
  
  std::ifstream stops_stream(stops_file);
  std::string line;
  getline(stops_stream, line);
  std::map<std::string, std::size_t> header = read_header(line);
  while (getline(stops_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements[1].front() == '"' && line_elements[2].back() == '"') { // small hack for Mexico City data
      line_elements[1].erase(0, 1);
      line_elements[2].pop_back();
      line_elements[1] = line_elements[1] + line_elements[2];
      line_elements.erase(line_elements.begin()+2);
    } if (line_elements.size() != 6) {
      std::cout << "Problems parsing stops" << std::endl;
      return 1;
    } std::string id = line_elements[header["stop_id"]];
    stops[id].id = id;
    stops[id].name = line_elements[header["stop_name"]];
    stops[id].lat = std::stod(line_elements[header["stop_lat"]]);
    stops[id].lon = std::stod(line_elements[header["stop_lon"]]);
    stops[id].zone = line_elements[header["zone_id"]];
    if (line_elements[header["wheelchair_boarding"]] == "1") stops[id].wheelchair_boarding = true;
    else stops[id].wheelchair_boarding = false;
  } stops_stream.close();
  std::cout << "\t" << stops.size() << " stops" << std::endl;
//  for (auto const &stop: stops) std::cout << "Stop[" << stop.second.id << "]: " << stop.second.name << " at " << stop.second.lat << ", " << stop.second.lon << std::endl;
  
  std::ifstream routes_stream(routes_file);
  getline(routes_stream, line);
  header = read_header(line);
  while (getline(routes_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n' || element.back() == ' ') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 7) {
      std::cout << "Problems parsing routes" << std::endl;
      return 1;
    } std::string id = line_elements[header["route_id"]];
    routes[id].id = id;
    routes[id].agency = line_elements[header["agency_id"]];
    routes[id].short_name = line_elements[header["route_short_name"]];
    routes[id].long_name = line_elements[header["route_long_name"]];
  } routes_stream.close();
  std::cout << "\t" << routes.size() << " routes" << std::endl;
//  for (auto const &route: routes) std::cout << "Route[" << route.second.id << "]: " << route.second.agency << " " << route.second.short_name << " is " << route.second.long_name << std::endl;
  
  std::ifstream agencies_stream(agencies_file);
  getline(agencies_stream, line);
  header = read_header(line);
  while (getline(agencies_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n' || element.back() == ' ') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 5) {
      std::cout << "Problems parsing agencies" << std::endl;
      return 1;
    } std::string id = line_elements[header["agency_id"]];
    agencies[id].id = id;
    agencies[id].name = line_elements[header["agency_name"]];
  } agencies_stream.close();
  std::cout << "\t" << agencies.size() << " agencies" << std::endl;
//  for (auto const &agency: agencies) std::cout << "Agency[" << agency.second.id << "]: " << agency.second.name << std::endl;
  
  std::ifstream services_stream(services_file);
  getline(services_stream, line);
  header = read_header(line);
  while (getline(services_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n' || element.back() == ' ') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 10) {
      std::cout << "Problems parsing services" << std::endl;
      return 1;
    } std::string id = line_elements[header["service_id"]];
    services[id].id = id;
    if (line_elements[header["monday"]] == "1") services[id].monday = true;
    else services[id].monday = false;
    if (line_elements[header["tuesday"]] == "1") services[id].tuesday = true;
    else services[id].tuesday = false;
    if (line_elements[header["wednesday"]] == "1") services[id].wednesday = true;
    else services[id].wednesday = false;
    if (line_elements[header["thursday"]] == "1") services[id].thursday = true;
    else services[id].thursday = false;
    if (line_elements[header["friday"]] == "1") services[id].friday = true;
    else services[id].friday = false;
    if (line_elements[header["saturday"]] == "1") services[id].saturday = true;
    else services[id].saturday = false;
    if (line_elements[header["sunday"]] == "1") services[id].sunday = true;
    else services[id].sunday = false;
  } services_stream.close();
  std::cout << "\t" << services.size() << " services" << std::endl;
//  for (auto const &service: services) std::cout << "Service[" << service.second.id << "]: M = " << service.second.monday << " Tu = " << service.second.tuesday << " W = " << service.second.wednesday << " Th = " << service.second.thursday << " F = " << service.second.friday << " Sa = " << service.second.saturday << " Su = " << service.second.sunday << std::endl;
  
  std::ifstream trips_stream(trips_file);
  getline(trips_stream, line);
  header = read_header(line);
  while (getline(trips_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n' || element.back() == ' ') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 7) {
      std::cout << "Problems parsing trips" << std::endl;
      return 1;
    } std::string id = line_elements[header["trip_id"]];
    trips[id].id = id;
    trips[id].route = line_elements[header["route_id"]];
    trips[id].service = line_elements[header["service_id"]];
    trips[id].head_sign = line_elements[header["trip_headsign"]];
    trips[id].short_name = line_elements[header["trip_short_name"]];
  } trips_stream.close();
  std::cout << "\t" << trips.size() << " trips" << std::endl;
//  for (auto const &trip: trips) std::cout << "Trip[" << trip.second.id << "] from route " << trip.second.route << " service " << trip.second.service << ": " << trip.second.head_sign << std::endl;
  
  std::ifstream frequencies_stream(frequencies_file);
  getline(frequencies_stream, line);
  header = read_header(line);
  while (getline(frequencies_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n' || element.back() == ' ') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 5) {
      std::cout << "Problems parsing frequencies" << std::endl;
      return 1;
    } std::stringstream start_stream(line_elements[header["start_time"]]), end_stream(line_elements[header["end_time"]]);
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
    std::string trip_id = line_elements[header["trip_id"]];
    trips[trip_id].frequencies.emplace_back();
    trips[trip_id].frequencies.back().start = start;
    trips[trip_id].frequencies.back().end = end;
    trips[trip_id].frequencies.back().headway = std::stoi(line_elements[header["headway_secs"]]);
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
  header = read_header(line);
  while (getline(stop_times_stream, line)) {
    std::stringstream line_stream(line);
    std::vector<std::string> line_elements;
    std::string element;
    while (getline(line_stream, element, ',')) {
      while (element.back() == '\r' || element.back() == '\n' || element.back() == ' ') element.pop_back();
      line_elements.push_back(element);
    } if (line_elements.size() != 6) {
      std::cout << "Problems parsing stop times" << std::endl;
      return 1;
    } std::string trip_id = line_elements[header["trip_id"]];
    std::stringstream arrival_stream(line_elements[header["arrival_time"]]), departure_stream(line_elements[header["departure_time"]]);
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
    unsigned int sequence = std::stoi(line_elements[header["stop_sequence"]]);
    trips[trip_id].stops[sequence].sequence = sequence;
    trips[trip_id].stops[sequence].arrival = arrival;
    trips[trip_id].stops[sequence].departure = departure;
    trips[trip_id].stops[sequence].stop = line_elements[header["stop_id"]];
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
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
  return 0;
}

void Isochrone_generator::create_hexes_for_stops(int hex_buffer_size) {
  std::cout << "Generating hexes within buffer of stops..." << std::endl;
  clock_t start_time = clock();
  
  for (auto const &stop: stops) {
    LatLng ll;
    ll.lat = degsToRads(stop.second.lat);
    ll.lng = degsToRads(stop.second.lon);
    H3Index hex;
    latLngToCell(&ll, h3_resolution, &hex);
    hexes[hex].stops.push_back(stop.first);
  }
  
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
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
}

void Isochrone_generator::create_mexico_city_starting_points() {
  std::cout << "Creating good starting points for Mexico City..." << std::endl;
  clock_t start_time = clock();
  const int same_station_search_size = 3;
  
  for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "METRO") continue;
    for (auto const &stop: trip.second.stops) {
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(same_station_search_size, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, same_station_search_size, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = "Metro";
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
      maxGridDiskSize(same_station_search_size, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, same_station_search_size, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = "Tren Ligero";
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
      maxGridDiskSize(same_station_search_size, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, same_station_search_size, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = "Suburbano";
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  } for (auto const &trip: trips) {
    if (routes[trip.second.route].agency != "MB") continue;
    if (routes[trip.second.route].short_name == "SE L12") continue; // servicio emergente L12
    for (auto const &stop: trip.second.stops) {
      if (stops[stop.second.stop].name == "Metro Coyuya") continue; // skip some bad names
      if (stops[stop.second.stop].name == "Glorieta Insurgentes") continue;
      if (stops[stop.second.stop].name == "Glorieta de Colón") continue;
      LatLng ll;
      H3Index hex;
      ll.lat = degsToRads(stops[stop.second.stop].lat);
      ll.lng = degsToRads(stops[stop.second.stop].lon);
      latLngToCell(&ll, h3_resolution, &hex);
      int64_t max_hexes;
      maxGridDiskSize(same_station_search_size, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, same_station_search_size, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = "Metrobús";
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
      maxGridDiskSize(same_station_search_size, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, same_station_search_size, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = "Cablebús";
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
      maxGridDiskSize(same_station_search_size, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex, same_station_search_size, hexes_within_distance);
      bool match_found = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes[hexes_within_distance[i]].stop_name == stops[stop.second.stop].name) match_found = true;
      } delete []hexes_within_distance;
      if (!match_found && hexes[hex].stop_name.empty()) {
        hexes[hex].transport_type = "Trolebús";
        hexes[hex].stop_name = stops[stop.second.stop].name;
      }
    }
  }
  
  for (auto &hex: hexes) {
    if (!hex.second.stop_name.empty()) hex.second.stop_name = hex.second.transport_type + " " + hex.second.stop_name;
  }
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
}

void Isochrone_generator::write_starting_points(std::string &starting_points_file) {
  std::cout << "Writing isochrone starting points..." << std::endl;
  clock_t start_time = clock();
  
  std::ofstream output_stream;
  output_stream.open(starting_points_file);
  nlohmann::json geojson;
  geojson["type"] = "FeatureCollection";
  geojson["features"] = nlohmann::json::array();
  for (auto const &hex: hexes) {
    if (hex.second.stop_name.empty()) continue;
    LatLng ll;
    cellToLatLng(hex.first, &ll);
    geojson["features"].push_back(nlohmann::json::object());
    geojson["features"].back()["type"] = "Feature";
    geojson["features"].back()["geometry"] = nlohmann::json::object();
    geojson["features"].back()["geometry"]["type"] = "Point";
    geojson["features"].back()["geometry"]["coordinates"] = {radsToDegs(ll.lng), radsToDegs(ll.lat)};
    geojson["features"].back()["properties"] = nlohmann::json::object();
    geojson["features"].back()["properties"]["id"] = std::to_string(hex.first);
    geojson["features"].back()["properties"]["system"] = hex.second.transport_type;
    geojson["features"].back()["properties"]["name"] = hex.second.stop_name;
  } output_stream << geojson.dump() << std::endl;
  output_stream.close();
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
}

void Isochrone_generator::add_walking_connections(std::string &osm, double walking_speed) {
  std::cout << "Adding walking connections..." << std::endl;
  clock_t start_time = clock();
  
  // Walking connections with OSM street network
  osmium::io::Reader reader{osm.c_str(), osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
  
  using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
  using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;
  index_type index;
  location_handler_type location_handler{index};

  RoadHandler handler;
  handler.hexes = &hexes;
  handler.h3_resolution = h3_resolution;
  osmium::apply(reader, location_handler, handler);
  reader.close();
  
  for (auto const &from_hex: handler.walking_connections) {
    for (auto const &to_hex: from_hex.second) {
      add_walking_connection(from_hex.first, to_hex, walking_speed);
    }
  }
  
  // One-hex tolerance to allow for hexes near roads + fewer holes
  std::vector<H3Index> hexes_to_connect;
  for (auto &hex: hexes) {
    bool has_walking_connection = false;
    for (auto const &connection: hex.second.connections) {
      if (connection.how == "walk") {
        has_walking_connection = true;
        break;
      }
    } if (!has_walking_connection) {
      int64_t max_hexes;
      maxGridDiskSize(1, &max_hexes);
      H3Index *hexes_within_distance = new H3Index[max_hexes];
      gridDisk(hex.first, 1, hexes_within_distance);
      bool border_hex = false;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes_within_distance[i] != 0 &&
            hexes.count(hexes_within_distance[i])) {
          for (auto const &connection: hexes[hexes_within_distance[i]].connections) {
            if (connection.how == "walk") {
              border_hex = true;
              break;
            }
          }
        }
      } delete []hexes_within_distance;
      if (border_hex) hexes_to_connect.push_back(hex.first);
    }
  } for (auto const &hex: hexes_to_connect) {
    int64_t max_hexes;
    maxGridDiskSize(1, &max_hexes);
    H3Index *hexes_within_distance = new H3Index[max_hexes];
    gridDisk(hex, 1, hexes_within_distance);
    LatLng from_ll;
    cellToLatLng(hex, &from_ll);
    for (int i = 0; i < max_hexes; ++i) {
      if (hexes_within_distance[i] != 0 && hexes.count(hexes_within_distance[i])) {
        LatLng to_ll;
        cellToLatLng(hexes_within_distance[i], &to_ll);
        double distance = greatCircleDistanceKm(&from_ll, &to_ll);
        
        hexes[hex].connections.emplace_back();
        hexes[hex].connections.back().to = hexes_within_distance[i];
        hexes[hex].connections.back().travel_time = distance/walking_speed;
        hexes[hex].connections.back().wait_time = 0.0;
        hexes[hex].connections.back().how = "walk";
        
        hexes[hexes_within_distance[i]].connections.emplace_back();
        hexes[hexes_within_distance[i]].connections.back().to = hex;
        hexes[hexes_within_distance[i]].connections.back().travel_time = distance/walking_speed;
        hexes[hexes_within_distance[i]].connections.back().wait_time = 0.0;
        hexes[hexes_within_distance[i]].connections.back().how = "walk";
      }
    } delete []hexes_within_distance;
  }
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
}

void Isochrone_generator::add_transit_connections() {
  std::cout << "Adding transit connections..." << std::endl;
  clock_t start_time = clock();
  
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
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
}

void Isochrone_generator::write_isochrones_for_starting_points(std::string &isochrones_folder, std::vector<double> &isochrone_times) {
  std::ofstream output_stream;
  for (auto const &hex: hexes) {
    if (hex.second.stop_name.empty()) continue;
    std::cout << "Computing and writing isochrone for " << hex.second.stop_name << "..." << std::endl;
    clock_t start_time = clock();
    auto time_and_previous = compute_routes_from_hex(hex.first);
    
    nlohmann::json geojson = create_isochrones_from_routes(time_and_previous.first, isochrone_times);
    geojson["properties"]["id"] = std::to_string(hex.first);
    geojson["properties"]["system"] = hex.second.transport_type;
    geojson["properties"]["name"] = hex.second.stop_name;
    output_stream.open(isochrones_folder + "/" + std::to_string(hex.first) + ".geojson");
    output_stream << geojson.dump() << std::endl;
    output_stream.close();
    
    std::cout << "\tdone in ";
    print_timer(start_time);
    std::cout << std::endl;
  }
}

int Isochrone_generator::write_hexes_gpkg(std::string &hexes_file) {
  std::cout << "Writing hexes..." << std::endl;
  clock_t start_time = clock();
  
  GDALAllRegister();
  const char *driver_name = "GPKG";
  GDALDriver *driver = GetGDALDriverManager()->GetDriverByName(driver_name);
  GDALDataset *output_dataset = (GDALDataset*) GDALOpenEx(hexes_file.c_str(), GDAL_OF_READONLY, NULL, NULL, NULL);
  if (output_dataset != NULL) {
    std::cout << "Overwriting file..." << std::endl;
    if (driver->Delete(hexes_file.c_str())!= CE_None) {
      std::cerr << "Error: Couldn't erase existing file to create output file" << std::endl;
      return 1;
    } GDALClose(output_dataset);
  } output_dataset = driver->Create(hexes_file.c_str(),0,0,0,GDT_Unknown,NULL);
  if (output_dataset == NULL) {
    std::cout << "Error: Could not create output file." << std::endl;
    return 1;
  } OGRSpatialReference spatial_reference;
  spatial_reference.importFromEPSG(4326);
  
  OGRLayer *hexes_layer = output_dataset->CreateLayer("hexes", &spatial_reference, wkbPolygon, NULL);
  if (hexes_layer == NULL) {
    std::cerr << "Error: Could not create hexes layer." << std::endl;
    return 1;
  } for (auto const &hex: hexes) {
    CellBoundary cb;
    cellToBoundary(hex.first, &cb);
    OGRFeature *hex_feature = OGRFeature::CreateFeature(hexes_layer->GetLayerDefn());
    OGRPolygon hex_polygon;
    OGRLinearRing hex_ring;
    for (int vertex = 0; vertex < cb.numVerts; ++vertex) hex_ring.addPoint(radsToDegs(cb.verts[vertex].lng),
                                                                           radsToDegs(cb.verts[vertex].lat));
    hex_ring.addPoint(radsToDegs(cb.verts[0].lng), radsToDegs(cb.verts[0].lat));
    hex_polygon.addRing(&hex_ring);
    hex_feature->SetGeometry(&hex_polygon);
    if (hexes_layer->CreateFeature(hex_feature) != OGRERR_NONE) {
      std::cerr << "Error: Could not create hex feature." << std::endl;
      return 1;
    } OGRFeature::DestroyFeature(hex_feature);
  }
  
  GDALClose(output_dataset);
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
  return 0;
}

int Isochrone_generator::write_connections_gpkg(std::string &connections_file) {
  std::cout << "Writing connections..." << std::endl;
  clock_t start_time = clock();
  
  GDALAllRegister();
  const char *driver_name = "GPKG";
  GDALDriver *driver = GetGDALDriverManager()->GetDriverByName(driver_name);
  GDALDataset *output_dataset = (GDALDataset*) GDALOpenEx(connections_file.c_str(), GDAL_OF_READONLY, NULL, NULL, NULL);
  if (output_dataset != NULL) {
    std::cout << "Overwriting file..." << std::endl;
    if (driver->Delete(connections_file.c_str())!= CE_None) {
      std::cerr << "Error: Couldn't erase existing file to create output file" << std::endl;
      return 1;
    } GDALClose(output_dataset);
  } output_dataset = driver->Create(connections_file.c_str(),0,0,0,GDT_Unknown,NULL);
  if (output_dataset == NULL) {
    std::cout << "Error: Could not create output file." << std::endl;
    return 1;
  } OGRSpatialReference spatial_reference;
  spatial_reference.importFromEPSG(4326);
  
  OGRLayer *connections_layer = output_dataset->CreateLayer("connections", &spatial_reference, wkbLineString, NULL);
  if (connections_layer == NULL) {
    std::cerr << "Error: Could not create connections layer." << std::endl;
    return 1;
  } OGRFieldDefn wait_time_field("wait_time", OFTReal);
  if (connections_layer->CreateField(&wait_time_field) != OGRERR_NONE) {
    std::cout << "Error: Could not create field wait_time." << std::endl;
    return 1;
  } OGRFieldDefn travel_time_field("travel_time", OFTReal);
  if (connections_layer->CreateField(&travel_time_field) != OGRERR_NONE) {
    std::cout << "Error: Could not create field travel_time." << std::endl;
    return 1;
  }
  for (auto const &hex: hexes) {
    for (auto const &connection: hex.second.connections) {
      OGRFeature *connection_feature = OGRFeature::CreateFeature(connections_layer->GetLayerDefn());
      connection_feature->SetField("wait_time", connection.wait_time);
      connection_feature->SetField("travel_time", connection.travel_time);
      OGRLineString connection_line_string;
      LatLng hex_ll, connection_ll;
      cellToLatLng(hex.first, &hex_ll);
      cellToLatLng(connection.to, &connection_ll);
      connection_line_string.addPoint(radsToDegs(hex_ll.lng), radsToDegs(hex_ll.lat));
      connection_line_string.addPoint(radsToDegs(connection_ll.lng), radsToDegs(connection_ll.lat));
      connection_feature->SetGeometry(&connection_line_string);
      if (connections_layer->CreateFeature(connection_feature) != OGRERR_NONE) {
        std::cerr << "Error: Could not create connection feature." << std::endl;
        return 1;
      } OGRFeature::DestroyFeature(connection_feature);
    }
  }
  
  GDALClose(output_dataset);
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
  return 0;
}

void Isochrone_generator::write_hexes_geojson(std::string &hexes_file) {
  std::cout << "Writing hexes..." << std::endl;
  clock_t start_time = clock();
  
  std::ofstream output_stream;
  output_stream.open(hexes_file);
  nlohmann::json geojson;
  geojson["type"] = "FeatureCollection";
  geojson["features"] = nlohmann::json::array();
  for (auto const &hex: hexes) {
//    if (hex.second.stop_name.empty()) continue;
    CellBoundary cb;
    cellToBoundary(hex.first, &cb);
    geojson["features"].push_back(nlohmann::json::object());
    geojson["features"].back()["type"] = "Feature";
    geojson["features"].back()["geometry"] = nlohmann::json::object();
    geojson["features"].back()["geometry"]["type"] = "Polygon";
    geojson["features"].back()["geometry"]["coordinates"] = nlohmann::json::array();
    geojson["features"].back()["geometry"]["coordinates"].push_back(nlohmann::json::array());
    for (int vertex = 0; vertex < cb.numVerts; ++vertex) {
      geojson["features"].back()["geometry"]["coordinates"].back().push_back({radsToDegs(cb.verts[vertex].lng), radsToDegs(cb.verts[vertex].lat)});
    }
  } output_stream << geojson.dump() << std::endl;
  output_stream.close();
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
}

void Isochrone_generator::write_connections_geojson(std::string &connections_file) {
  std::cout << "Writing connections..." << std::endl;
  clock_t start_time = clock();
  
  std::ofstream output_stream;
  output_stream.open(connections_file);
  nlohmann::json geojson;
  geojson["type"] = "FeatureCollection";
  geojson["features"] = nlohmann::json::array();
  for (auto const &hex: hexes) {
    for (auto const &connection: hex.second.connections) {
      geojson["features"].push_back(nlohmann::json::object());
      geojson["features"].back()["type"] = "Feature";
      geojson["features"].back()["geometry"] = nlohmann::json::object();
      geojson["features"].back()["geometry"]["type"] = "LineString";
      geojson["features"].back()["geometry"]["coordinates"] = nlohmann::json::array();
      LatLng hex_ll, connection_ll;
      cellToLatLng(hex.first, &hex_ll);
      cellToLatLng(connection.to, &connection_ll);
      geojson["features"].back()["geometry"]["coordinates"].push_back({radsToDegs(hex_ll.lng), radsToDegs(hex_ll.lat)});
      geojson["features"].back()["geometry"]["coordinates"].push_back({radsToDegs(connection_ll.lng), radsToDegs(connection_ll.lat)});
      geojson["features"].back()["properties"] = nlohmann::json::object();
      geojson["features"].back()["properties"]["wait_time"] = connection.wait_time;
      geojson["features"].back()["properties"]["travel_time"] = connection.travel_time;
    }
  } output_stream << geojson.dump() << std::endl;
  output_stream.close();
  
  std::cout << "\tdone in ";
  print_timer(start_time);
  std::cout << std::endl;
}
