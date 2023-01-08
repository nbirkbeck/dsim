#include "trip_stats.h"

#include <iostream>

void TripStats::AddObservation(const std::string& src_name,
                               const std::string& dest_name,
                               double elapsed_time) {
  Info& info = data_[src_name][dest_name];
  info.total_time += elapsed_time;
  info.num_observations++;
}

double TripStats::GetAverageTripTime() {
  double average_time = 0;
  int num_pairs = 0;
  for (const auto& src : data_) {
    for (const auto& dest : src.second) {
      const double estimate =
          dest.second.total_time / std::max(1, dest.second.num_observations);
      average_time += estimate;
      num_pairs++;
    }
  }
  return average_time / num_pairs;
}

void TripStats::Print() {
  double average_time = 0;
  int num_pairs = 0;
  std::cout << "------------------------------------------------------------\n";
  for (const auto& src : data_) {
    for (const auto& dest : src.second) {
      const double estimate =
          dest.second.total_time / std::max(1, dest.second.num_observations);
      std::cout << src.first << " " << dest.first << " " << estimate
                << std::endl;
      average_time += estimate;
      num_pairs++;
    }
  }
  if (num_pairs)
    std::cout << average_time / num_pairs << "(" << num_pairs << ")\n";
}
