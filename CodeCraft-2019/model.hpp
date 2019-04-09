/*
 * model.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _MODEL_HPP_
#define _MODEL_HPP_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>

/*{{{ RawCar, RawRoad, RawCross. Probably require to modify */
struct RawCar {
  int id, from, to, speed, plan_time, priority, preset;
};

struct RawRoad {
  int id, len, speed, channel, from, to, is_duplex;
};

struct RawCross {
  int id, r1, r2, r3, r4;
};
/*}}}*/

struct Feedback {
  int start_time;
  std::vector<int> pre_path;
  double cost;
  int suggest_start_time;
};

struct Info2Point {
  int start_time;
  int from;
  int to;
  int speed;
};

struct InfoRoad {
  int len;
  int max_capacity;
  int limit_speed;

  // time_stamp : current_num_of_cars
  std::unordered_map<int, int> live_time_window;
};

struct PassBy {
  int start, end;
  int leave;
};

struct InfoPass {
  int start_time;
  std::vector<PassBy> pass_by;
};

/*{{{ class Graph : store adjacency list */
class Graph {
public:
  Graph(std::size_t sz)
    : size_(sz) { this->adjacency_.resize(sz); }
  ~Graph() {}
  std::size_t get_size() const;

  bool add_edge(const std::size_t u, const std::size_t v, const bool dir);

private:
  Graph() = default;
  std::size_t size_;
  std::vector<std::vector<int>> adjacency_;
};

inline std::size_t
Graph::get_size()
  const
{
  return this->size_;
}

inline bool
Graph::add_edge(const std::size_t u,
                const std::size_t v,
                const bool dir)
{
  if (u >= this->size_ || v >= this->size_) {
    return false;
  }

  this->adjacency_[u].push_back(v);
  if (!dir) {
    this->adjacency_[v].push_back(u);
  }

  return true;
}
/*}}}*/

class Model {
public:
  // Model(input data) ?
  ~Model() {}
  // TODO: dijkstra
  Feedback dijkstra(Info2Point &in2p);
  void update_roads_info(InfoPass &inps); // XXX: update roads_history_

private:
  Model() = default;

  std::vector<RawCar>      raw_cars_;
  std::vector<RawRoad>    raw_roads_;
  std::vector<RawCross> raw_crosses_;

  std::map<std::pair<int, int>, RawRoad*> uv_to_roads_;

  // (c1, c2) --> {0: .., 1: .., ... n: .. } updating after calling schedule.
  Graph graph_;
  std::map<std::pair<int, int>, InfoRoad> roads_history_;
};

#endif // ifndef _MODEL_HPP_
