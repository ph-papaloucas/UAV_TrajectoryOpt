#pragma once
#include "utils.hpp"
#include <array>
#include <vector>

class SimulationResult {
public:
  struct Snapshot {
    double timestamp;
    State state;
    Control control;
    ForcesMoments forces_moments;
  };

  // Add a snapshot to history
  void add_snapshot(double t, const State &state, const Control &control,
                    const ForcesMoments &fm) {
    history_.push_back({t, state, control, fm});
  }

  // Getters
  const std::vector<Snapshot> &get_history() const { return history_; }
  int size() const { return history_.size(); }
  bool empty() const { return history_.empty(); }

  // Access specific snapshot
  const Snapshot &operator[](int index) const { return history_[index]; }

  // Clear history
  void clear() { history_.clear(); }

  // Get just states, controls, etc. (for plotting)
  std::vector<State> get_states() const {
    std::vector<State> states;
    for (const auto &snap : history_) {
      states.push_back(snap.state);
    }
    return states;
  }

  std::vector<double> get_timestamps() const {
    std::vector<double> timestamps;
    for (const auto &snap : history_) {
      timestamps.push_back(snap.timestamp);
    }
    return timestamps;
  }

  // Add this method to your SimulationResult class:
  void save_to_csv(const std::string &filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open file: " + filename);
    }

    // Write header
    file << "time,px,py,pz,u,v,w,phi,theta,psi,p,q,r,"
         << "de,da,dr,throttle,"
         << "fx,fy,fz,l,m,n\n";

    // Write data
    for (const auto &snap : history_) {
      file << snap.timestamp << ",";

      // State: position, velocity, attitude, rates
      for (int i = 0; i < 12; ++i) {
        file << snap.state[i] << (i < 11 ? "," : "");
      }

      // Controls
      file << ",";
      for (int i = 0; i < 4; ++i) {
        file << snap.control[i] << (i < 3 ? "," : "");
      }

      // Forces and moments
      file << ",";
      for (int i = 0; i < 6; ++i) {
        file << snap.forces_moments[i] << (i < 5 ? "," : "");
      }

      file << "\n";
    }

    file.close();
  }

private:
  std::vector<Snapshot> history_;
};