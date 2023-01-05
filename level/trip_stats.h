#ifndef DSIM_TRIP_STATS_H_
#define DSIM_TRIP_STATS_H_ 1

#include <string>
#include <unordered_map>

class TripStats {
public:
  struct Info {
    double total_time = 0;
    int num_observations = 0;
  };

  void AddObservation(const std::string& src_name,
                      const std::string& dest_name,
                      double elapsed_time);
  double GetAverageTripTime();
  void Print();

 private:
  std::unordered_map<std::string, std::unordered_map<std::string, Info> > data_;
};

#endif
