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

void Isochrone_generator::add_transit_line(const char *system, const char *line, std::vector<Stop> &stops, double total_time, double frequency) {
  
  // Add stops
  for (auto const &stop: stops) {
    if (stop.name == "Ciudad Azteca" ||
        stop.name == "Lechería" ||
        stop.name == "Pantitlán" ||
        stop.name == "Calle 6") continue;
    LatLng ll;
    H3Index hex;
    ll.lat = degsToRads(stop.lat);
    ll.lng = degsToRads(stop.lon);
    latLngToCell(&ll, h3_resolution, &hex);
    if (this->hexes[hex].stop_name.empty()) {
      this->hexes[hex].transport_type = system;
      this->hexes[hex].stop_name = stop.name;
    }
  }
  
  // Compute total length
  double total_length = 0.0;
  std::vector<Stop>::const_iterator prev_stop = stops.begin();
  std::vector<Stop>::const_iterator next_stop = prev_stop;
  ++next_stop;
  while (next_stop != stops.end()) {
    LatLng prev_ll, next_ll;
    prev_ll.lat = degsToRads(prev_stop->lat);
    prev_ll.lng = degsToRads(prev_stop->lon);
    next_ll.lat = degsToRads(next_stop->lat);
    next_ll.lng = degsToRads(next_stop->lon);
    total_length += greatCircleDistanceKm(&prev_ll, &next_ll);
    ++prev_stop;
    ++next_stop;
  }
  
  // Add connections
  prev_stop = stops.begin();
  next_stop = prev_stop;
  ++next_stop;
  while (next_stop != stops.end()) {
    LatLng prev_ll, next_ll;
    prev_ll.lat = degsToRads(prev_stop->lat);
    prev_ll.lng = degsToRads(prev_stop->lon);
    next_ll.lat = degsToRads(next_stop->lat);
    next_ll.lng = degsToRads(next_stop->lon);
    double this_length = greatCircleDistanceKm(&prev_ll, &next_ll);
    H3Index prev_hex, next_hex;
    latLngToCell(&prev_ll, h3_resolution, &prev_hex);
    latLngToCell(&next_ll, h3_resolution, &next_hex);
//    std::cout << "Adding connection between " << prev_stop->name << " and " << next_stop->name << " in " << total_time*this_length/total_length; << " seconds (" << this_length << " km)" << std::endl;
    this->hexes[prev_hex].connections.emplace_back();
    this->hexes[prev_hex].connections.back().to = next_hex;
    this->hexes[prev_hex].connections.back().travel_time = (total_time/3600.0)*(this_length/total_length);
    this->hexes[prev_hex].connections.back().wait_time = frequency/3600.0;
    this->hexes[next_hex].connections.emplace_back();
    this->hexes[next_hex].connections.back().to = prev_hex;
    this->hexes[next_hex].connections.back().travel_time = (total_time/3600.0)*(this_length/total_length);
    this->hexes[next_hex].connections.back().wait_time = frequency/3600.0;
    ++prev_stop;
    ++next_stop;
  }
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

void Isochrone_generator::add_more_stops() {
  std::vector<Stop> mxb1, mxb1xp, mxb1aifa, mxb2, mxb2xp, mxb3, mxb3xp1, mxb3xp2, mxb4, mxb4xp, mxcb1;
  mxb1.emplace_back("Ciudad Azteca", 19.53439836977682, -99.02729433667595);
  mxb1.emplace_back("Quinto Sol", 19.53924959962029, -99.02518429559673);
  mxb1.emplace_back("Josefa Ortíz", 19.5462272854162, -99.02211335535597);
  mxb1.emplace_back("Industrial", 19.55015230075902, -99.02051893763851);
  mxb1.emplace_back("UNITEC", 19.55378623821462, -99.0194231139581);
  mxb1.emplace_back("Alfredo Torres", 19.55816831329086, -99.01728201710492);
  mxb1.emplace_back("Zodiaco", 19.56322833706457, -99.01514464210868);
  mxb1.emplace_back("Adolfo López Mateos", 19.5672998728123, -99.01513557582894);
  mxb1.emplace_back("Vocacional 3", 19.569370586861, -99.01703869096195);
  mxb1.emplace_back("Valle de Ecatepec", 19.57300898347939, -99.02048433220071);
  mxb1.emplace_back("Las Américas L1", 19.58058755499126, -99.02404114210755);
  mxb1.emplace_back("Primero de Mayo", 19.58754925094757, -99.02441910053915);
  mxb1.emplace_back("Hospital", 19.59450828399905, -99.01990951722591);
  mxb1.emplace_back("Aquiles Serdán", 19.59851902831778, -99.0178880654198);
  mxb1.emplace_back("Jardines de Morelos", 19.60032312748448, -99.01599789882803);
  mxb1.emplace_back("Palomas", 19.60732425456538, -99.01148972018244);
  mxb1.emplace_back("19 de Septiembre", 19.61080505035884, -99.00909868729524);
  mxb1.emplace_back("Central de Abastos", 19.61696824341515, -99.00856280290067);
  mxb1.emplace_back("Insurgentes", 19.62575892998876, -99.00318088554235);
  mxb1.emplace_back("Hidalgo", 19.6334352456974, -99.00160047332518);
  mxb1.emplace_back("Cuauhtémoc Sur", 19.6381742183197, -99.00172772075214);
  mxb1.emplace_back("Cuauhtémoc Norte", 19.64626385345605, -99.00408373206334);
  mxb1.emplace_back("Esmeralda", 19.65218340008285, -99.00443591515533);
  mxb1.emplace_back("Ojo de Agua", 19.66138354043803, -99.00083737876062);
  add_transit_line("Mexibús", "MXB1", mxb1, 54*60, 10*60);
  
  mxb1xp.emplace_back("Ciudad Azteca", 19.53439836977682, -99.02729433667595);
  mxb1xp.emplace_back("UNITEC", 19.55378623821462, -99.0194231139581);
  mxb1xp.emplace_back("Vocacional 3", 19.569370586861, -99.01703869096195);
  mxb1xp.emplace_back("Primero de Mayo", 19.58754925094757, -99.02441910053915);
  mxb1xp.emplace_back("Hospital", 19.59450828399905, -99.01990951722591);
  mxb1xp.emplace_back("Jardines de Morelos", 19.60032312748448, -99.01599789882803);
  mxb1xp.emplace_back("Central de Abastos", 19.61696824341515, -99.00856280290067);
  mxb1xp.emplace_back("Cuauhtémoc Norte", 19.64626385345605, -99.00408373206334);
  mxb1xp.emplace_back("Ojo de Agua", 19.66138354043803, -99.00083737876062);
  add_transit_line("Mexibús", "MXB1XP", mxb1xp, 26*60, 20*60);
  
  mxb1aifa.emplace_back("Ojo de Agua", 19.66138354043803, -99.00083737876062);
  mxb1aifa.emplace_back("Loma Bonita", 19.66832570410111, -99.0040321986511);
  mxb1aifa.emplace_back("Ozumbilla", 19.68093484514425, -98.99588241650437);
  mxb1aifa.emplace_back("San Francisco", 19.68846350395874, -98.98816218091812);
  mxb1aifa.emplace_back("Tecámac", 19.71159736556294, -98.97498296892708);
  mxb1aifa.emplace_back("Glorieta Militar", 19.74952338534079, -98.97740534118537);
  mxb1aifa.emplace_back("Combustibles", 19.76261949803738, -98.99291458514243);
  mxb1aifa.emplace_back("Hacienda", 19.75366155831502, -99.00407422849152);
  mxb1aifa.emplace_back("Terminal de Pasajeros", 19.73618584511916, -99.02720019700827);
  add_transit_line("Mexibús", "MXB1AIFA", mxb1aifa, 25*60, 10*60);

  mxb2.emplace_back("Las Américas L2", 19.58971950628505, -99.02343120858322);
  mxb2.emplace_back("1o de Mayo L2", 19.59340855236048, -99.02657138050004);
  mxb2.emplace_back("San Martín", 19.59574339260342, -99.0288392261936);
  mxb2.emplace_back("Puente de Fierro", 19.60077346514725, -99.03304489436023);
  mxb2.emplace_back("Casa de Morelos", 19.59935669742654, -99.03191482016831);
  mxb2.emplace_back("UPE", 19.60197359838236, -99.04149266291061);
  mxb2.emplace_back("San Cristobal", 19.60460709854985, -99.04529975164978);
  mxb2.emplace_back("Agricultura", 19.60708012695657, -99.04926706900952);
  mxb2.emplace_back("ISSEMYM", 19.60690729343817, -99.05130415999449);
  mxb2.emplace_back("El Carmen", 19.60906585295757, -99.054385079996);
  mxb2.emplace_back("Ecatepec", 19.61121871875001, -99.05755303594596);
  mxb2.emplace_back("DIF", 19.61448132731259, -99.05987881778697);
  mxb2.emplace_back("Guadalupe Victoria", 19.61914939374332, -99.06612441598996);
  mxb2.emplace_back("Venustiano Carranza", 19.62030288546155, -99.0677673080465);
  mxb2.emplace_back("FOVISSSTE", 19.62165626508903, -99.07008157116425);
  mxb2.emplace_back("San Carlos", 19.62507103931515, -99.0752641409198);
  mxb2.emplace_back("La Laguna", 19.627014165393, -99.08025535039479);
  mxb2.emplace_back("Parque Residencial", 19.63016610033181, -99.08482610748759);
  mxb2.emplace_back("Eje 8", 19.6323809023466, -99.09185506301377);
  mxb2.emplace_back("1ra. de Villa", 19.6331227100971, -99.09369383661803);
  mxb2.emplace_back("Las Flores Zacuatitla", 19.63415725768794, -99.09660459690936);
  mxb2.emplace_back("San Francisco", 19.63620633116745, -99.10202815386492);
  mxb2.emplace_back("Héroes Canosas", 19.63590856843284, -99.10561565881612);
  mxb2.emplace_back("Coacalco Tultepec", 19.63510546046314, -99.11276325762481);
  mxb2.emplace_back("Ex-hacienda San Felipe", 19.63386415837256, -99.1156403377791);
  mxb2.emplace_back("Bosques del Valle", 19.63319379452796, -99.11726181380475);
  mxb2.emplace_back("Coacalco Berriozábal", 19.63084969239678, -99.12447587798128);
  mxb2.emplace_back("Santa María", 19.63072148633727, -99.12722826849203);
  mxb2.emplace_back("Villas de San José", 19.62987359831654, -99.13040249323046);
  mxb2.emplace_back("Mariscala Real del Bosque", 19.62846231627136, -99.13571323267593);
  mxb2.emplace_back("Fuentes del Valle", 19.62820450529153, -99.13959079483186);
  mxb2.emplace_back("De la Cruz San Mateo", 19.62587956019357, -99.14872841273146);
  mxb2.emplace_back("Cartagena", 19.62504686998676, -99.15249877421446);
  mxb2.emplace_back("Bello Horizonte", 19.62328845444674, -99.15900262607263);
  mxb2.emplace_back("Bandera Tultitlán", 19.6222629022576, -99.16294571421722);
  mxb2.emplace_back("Buenavista", 19.62096099470737, -99.16728414039603);
  mxb2.emplace_back("COCEM", 19.62005274899639, -99.17061115813237);
  mxb2.emplace_back("Recursos Hidráulicos", 19.61749204344672, -99.1780422328082);
  mxb2.emplace_back("Chilpan", 19.61535264865363, -99.17971943441471);
  mxb2.emplace_back("Ciudad Labor", 19.60917872788961, -99.18081689876557);
  mxb2.emplace_back("Vidriera", 19.606592174932, -99.18164084074678);
  mxb2.emplace_back("Lechería", 19.59862658661841, -99.18311624034162);
  mxb2.emplace_back("La Quebrada", 19.59212188408037, -99.18783419362067);
  add_transit_line("Mexibús", "MXB2", mxb2, 81*60, 15*60);

  mxb2xp.emplace_back("Ecatepec", 19.61121871875001, -99.05755303594596);
  mxb2xp.emplace_back("DIF", 19.61448132731259, -99.05987881778697);
  mxb2xp.emplace_back("Guadalupe Victoria", 19.61914939374332, -99.06612441598996);
  mxb2xp.emplace_back("FOVISSSTE", 19.62165626508903, -99.07008157116425);
  mxb2xp.emplace_back("La Laguna", 19.627014165393, -99.08025535039479);
  mxb2xp.emplace_back("Parque Residencial", 19.63016610033181, -99.08482610748759);
  mxb2xp.emplace_back("Eje 8", 19.6323809023466, -99.09185506301377);
  mxb2xp.emplace_back("1ra. de Villa", 19.6331227100971, -99.09369383661803);
  mxb2xp.emplace_back("Las Flores Zacuatitla", 19.63415725768794, -99.09660459690936);
  mxb2xp.emplace_back("San Francisco", 19.63620633116745, -99.10202815386492);
  mxb2xp.emplace_back("Héroes Canosas", 19.63590856843284, -99.10561565881612);
  mxb2xp.emplace_back("Coacalco Tultepec", 19.63510546046314, -99.11276325762481);
  mxb2xp.emplace_back("Coacalco Berriozábal", 19.63084969239678, -99.12447587798128);
  mxb2xp.emplace_back("Villas de San José", 19.62987359831654, -99.13040249323046);
  mxb2xp.emplace_back("Mariscala Real del Bosque", 19.62846231627136, -99.13571323267593);
  mxb2xp.emplace_back("Fuentes del Valle", 19.62820450529153, -99.13959079483186);
  mxb2xp.emplace_back("Cartagena", 19.62504686998676, -99.15249877421446);
  mxb2xp.emplace_back("Bandera Tultitlán", 19.6222629022576, -99.16294571421722);
  mxb2xp.emplace_back("Chilpan", 19.61535264865363, -99.17971943441471);
  mxb2xp.emplace_back("Lechería", 19.59862658661841, -99.18311624034162);
  add_transit_line("Mexibús", "MXB2XP", mxb2, 45*60, 15*60);
  
  mxb3.emplace_back("Pantitlán", 19.41622562060524, -99.0717413205666);
  mxb3.emplace_back("Calle 6", 19.4209852181542, -99.05884829820698);
  mxb3.emplace_back("El Barquito", 19.42012625450993, -99.04840132630977);
  mxb3.emplace_back("Maravillas", 19.41964571307251, -99.04413194343167);
  mxb3.emplace_back("Vicente Riva Palacio", 19.41821532751105, -99.04121235572096);
  mxb3.emplace_back("Virgencitas", 19.41519634388662, -99.03329607034728);
  mxb3.emplace_back("Nezahualcóyotl", 19.41415389170354, -99.03210025419071);
  mxb3.emplace_back("Lago de Chapala", 19.41201069713294, -99.02704994951381);
  mxb3.emplace_back("Adolfo López Mateos", 19.40919346592051, -99.02245280813919);
  mxb3.emplace_back("Palacio Municipal", 19.40727260344348, -99.01827037292249);
  mxb3.emplace_back("Sor Juana Inés de la Cruz", 19.40583943574982, -99.0138569239773);
  mxb3.emplace_back("El Castillito", 19.40472039095053, -99.01133966497878);
  mxb3.emplace_back("General Vicente Villada", 19.40270081890758, -99.00836209386476);
  mxb3.emplace_back("Rayito de Sol", 19.40649921294924, -99.00384764924844);
  mxb3.emplace_back("Las Mañanitas", 19.41054472777601, -99.00192616482502);
  mxb3.emplace_back("Rancho Grande", 19.41506495558606, -98.9998275465898);
  mxb3.emplace_back("Bordo de Xochiaca", 19.41554328164503, -98.99509364891462);
  mxb3.emplace_back("Las Torres", 19.41422081931805, -98.98900128084165);
  mxb3.emplace_back("Guerrero Chimalli", 19.41295216119731, -98.98258629638819);
  mxb3.emplace_back("Las Flores", 19.41452170344307, -98.97850788980891);
  mxb3.emplace_back("Canteros", 19.41851136564861, -98.97638248261553);
  mxb3.emplace_back("La Presa", 19.42212647188211, -98.97241708078207);
  mxb3.emplace_back("Embarcadero", 19.42532624894452, -98.96704680094442);
  mxb3.emplace_back("Santa Elena", 19.42854718448196, -98.96267147008716);
  mxb3.emplace_back("Ignacio Manuel Altamirano", 19.43139553230715, -98.95710027073898);
  mxb3.emplace_back("San Pablo", 19.43204785629158, -98.95155154640979);
  mxb3.emplace_back("Los Patos", 19.43128237723586, -98.94670730369472);
  mxb3.emplace_back("Refugio", 19.4302942317651, -98.94085234466959);
  mxb3.emplace_back("Chimalhuacán", 19.42381404525305, -98.93576830513244);
  add_transit_line("Mexibús", "MXB3", mxb3, 59*60, 10*60);
  
  mxb3xp1.emplace_back("Pantitlán", 19.41622562060524, -99.0717413205666);
  mxb3xp1.emplace_back("El Barquito", 19.42012625450993, -99.04840132630977);
  mxb3xp1.emplace_back("Vicente Riva Palacio", 19.41821532751105, -99.04121235572096);
  mxb3xp1.emplace_back("Nezahualcóyotl", 19.41415389170354, -99.03210025419071);
  mxb3xp1.emplace_back("Adolfo López Mateos", 19.40919346592051, -99.02245280813919);
  mxb3xp1.emplace_back("Palacio Municipal", 19.40727260344348, -99.01827037292249);
  mxb3xp1.emplace_back("General Vicente Villada", 19.40270081890758, -99.00836209386476);
  mxb3xp1.emplace_back("Rayito de Sol", 19.40649921294924, -99.00384764924844);
  mxb3xp1.emplace_back("Las Torres", 19.41422081931805, -98.98900128084165);
  mxb3xp1.emplace_back("Las Flores", 19.41452170344307, -98.97850788980891);
  mxb3xp1.emplace_back("La Presa", 19.42212647188211, -98.97241708078207);
  mxb3xp1.emplace_back("Los Patos", 19.43128237723586, -98.94670730369472);
  mxb3xp1.emplace_back("Acuitlapilco", 19.42953228900163, -98.93868332308342);
  add_transit_line("Mexibús", "MXB3XP1", mxb3xp1, 33*60, 15*60);
  
  mxb3xp2.emplace_back("Pantitlán", 19.41622562060524, -99.0717413205666);
  mxb3xp2.emplace_back("Vicente Riva Palacio", 19.41821532751105, -99.04121235572096);
  mxb3xp2.emplace_back("Adolfo López Mateos", 19.40919346592051, -99.02245280813919);
  mxb3xp2.emplace_back("General Vicente Villada", 19.40270081890758, -99.00836209386476);
  mxb3xp2.emplace_back("Las Torres", 19.41422081931805, -98.98900128084165);
  mxb3xp2.emplace_back("La Presa", 19.42212647188211, -98.97241708078207);
  mxb3xp2.emplace_back("Los Patos", 19.43128237723586, -98.94670730369472);
  mxb3xp2.emplace_back("Chimalhuacán", 19.42381404525305, -98.93576830513244);
  add_transit_line("Mexibús", "MXB3XP2", mxb3xp2, 24*60, 15*60);
  
  mxb4.emplace_back("Indios Verdes", 19.4977082808276, -99.11826439909025);
  mxb4.emplace_back("Periférico", 19.51783131791117, -99.09897959343601);
  mxb4.emplace_back("Martín Carrera", 19.52097490393932, -99.09295842001298);
  mxb4.emplace_back("Clínica 76", 19.52707584965583, -99.08241931160735);
  mxb4.emplace_back("Vía Morelos", 19.53078534788809, -99.07557497701603);
  mxb4.emplace_back("Monumento a Morelos", 19.53257570587735, -99.07139857876196);
  mxb4.emplace_back("5 de Febrero", 19.53511557267713, -99.06488878233863);
  mxb4.emplace_back("Santa Clara", 19.5390581429744, -99.05914401896406);
  mxb4.emplace_back("Cerro Gordo", 19.54340210416298, -99.05553659467225);
  mxb4.emplace_back("Servicios Administrativos", 19.54710247009872, -99.05269454952266);
  mxb4.emplace_back("Clínica 93", 19.55098374663261, -99.05189359866041);
  mxb4.emplace_back("Industrial", 19.55748425159363, -99.04928703253776);
  mxb4.emplace_back("5ta. Aparición", 19.56256976254083, -99.04675252633031);
  mxb4.emplace_back("Tulpetlac", 19.5679193735473, -99.04423038988651);
  mxb4.emplace_back("Siervo de la Nación", 19.57150421629035, -99.04319831116972);
  mxb4.emplace_back("Nuevo Laredo", 19.57683531905458, -99.04283223412033);
  mxb4.emplace_back("Laureles", 19.58267936790922, -99.04177919606445);
  mxb4.emplace_back("La Viga", 19.59212911561394, -99.03933620502499);
  mxb4.emplace_back("San Cristobal", 19.59669137531058, -99.03721690646914);
  mxb4.emplace_back("Puente de Fierro", 19.60077346514725, -99.03304489436023);
  mxb4.emplace_back("Izcalli Palomas", 19.61262582522854, -99.01934955043856);
  mxb4.emplace_back("Central de Abastos", 19.61696824341515, -99.00856280290067);
  mxb4.emplace_back("Ejido Santo Tomás", 19.63019119676257, -99.00730593483017);
  mxb4.emplace_back("Revolución", 19.63114650828414, -99.01494539749635);
  mxb4.emplace_back("Margarito F. Ayala", 19.63198399653626, -99.01945615545702);
  mxb4.emplace_back("Flores", 19.63320041939916, -99.02828511210801);
  mxb4.emplace_back("Bosques", 19.63436128429021, -99.03504072318066);
  mxb4.emplace_back("Universidad Mexiquense del Bicentenario", 19.63646855282241, -99.04712980635833);
  add_transit_line("Mexibús", "MXB4", mxb4, 73*60, 10*60);
  
  mxb4xp.emplace_back("Indios Verdes", 19.4977082808276, -99.11826439909025);
  mxb4xp.emplace_back("Periférico", 19.51783131791117, -99.09897959343601);
  mxb4xp.emplace_back("Clínica 76", 19.52707584965583, -99.08241931160735);
  mxb4xp.emplace_back("Santa Clara", 19.5390581429744, -99.05914401896406);
  mxb4xp.emplace_back("Clínica 93", 19.55098374663261, -99.05189359866041);
  mxb4xp.emplace_back("Tulpetlac", 19.5679193735473, -99.04423038988651);
  mxb4xp.emplace_back("Nuevo Laredo", 19.57683531905458, -99.04283223412033);
  mxb4xp.emplace_back("Puente de Fierro", 19.60077346514725, -99.03304489436023);
  mxb4xp.emplace_back("Central de Abastos", 19.61696824341515, -99.00856280290067);
  mxb4xp.emplace_back("Universidad Mexiquense del Bicentenario", 19.63646855282241, -99.04712980635833);
  add_transit_line("Mexibús", "MXB4XP", mxb4xp, 45*60, 15*60);
  
  mxcb1.emplace_back("Santa Clara", 19.54032305676051, -99.05927329249823);
  mxcb1.emplace_back("Hank González", 19.54957524296558, -99.07377549824112);
  mxcb1.emplace_back("Fátima", 19.55383916088869, -99.07708890110611);
  mxcb1.emplace_back("Tablas del Pozo", 19.55693081936162, -99.07976596106869);
  mxcb1.emplace_back("Los Bordos", 19.56266808806356, -99.0840018497678);
  mxcb1.emplace_back("Deportivo", 19.56611476914636, -99.08784414761858);
  mxcb1.emplace_back("La Cañada", 19.56848141968011, -99.09146944517855);
  add_transit_line("Mexicable", "MXCB1", mxcb1, 17*60, 1*60);
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
  
  // Remove thin holes
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
      int connected_neighbours = 0;
      for (int i = 0; i < max_hexes; ++i) {
        if (hexes_within_distance[i] != 0 &&
            hexes.count(hexes_within_distance[i])) {
          bool connected_neighbour = false;
          for (auto const &connection: hexes[hexes_within_distance[i]].connections) {
            if (connection.how == "walk") {
              connected_neighbour = true;
              break;
            }
          } if (connected_neighbour) ++connected_neighbours;
        }
      } delete []hexes_within_distance;
      if (connected_neighbours >= 5) hexes_to_connect.push_back(hex.first);
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
