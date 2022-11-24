#ifndef Data_model_h
#define Data_model_h

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

#endif /* Data_model_h */
