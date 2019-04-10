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
#include <functional> // std::function
#include <unordered_map>
#include <utility>    // std::pair

/*{{{ DEFINE MACRO */
#define   CAR_ID                  0
#define   CAR_FROM                1
#define   CAR_TO                  2
#define   CAR_SPEED               3
#define   CAR_PLAN_TIME           4
#define   CAR_PRIORITY            5
#define   CAR_PRESET              6

#define   ROAD_ID                 0
#define   ROAD_LEN                1
#define   ROAD_SPEED              2
#define   ROAD_CHANNEL            3
#define   ROAD_FROM               4
#define   ROAD_TO                 5
#define   ROAD_IS_DUPLEX          6

#define   CROSS_ID                0
#define   CROSS_UP                1
#define   CROSS_RIGHT             2
#define   CROSS_DOWN              3
#define   CROSS_LEFT              4

#define   PRESET_CAR_ID           0
#define   PRESET_CAR_START_TIME   1
#define   PRESET_CAR_ROAD_START   2
/*}}}*/

/*{{{ RawCar, RawRoad, RawCross.*/
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
  // int start_time, sugeest_start_time;
  std::vector<int> t_path;
  int cost_time;
};

struct StartEndInfo {
  int start_time, from_index, to_index, speed;
};

struct NodeInfo {
  int index, cost_time, volumn;
};

struct RoadInfo {
  int id, len, speed, channel;
};

/*{{{ @Deprecated. class Graph : store adjacency list */
/*
 * class Graph {
 * public:
 *   Graph(std::size_t sz)
 *     : size_(sz) { this->adjacency_.resize(sz); }
 *   ~Graph() {}
 *   std::size_t get_size() const;
 *   std::vector<int> get_adjacency(std::size_t u) const;
 *   bool add_edge(const std::size_t u, const std::size_t v, const bool dir);
 * 
 * private:
 *   Graph() = default;
 *   std::size_t size_;
 *   std::vector<std::vector<int>> adjacency_;
 * };
 * 
 * inline std::size_t
 * Graph::get_size()
 *   const
 * {
 *   return this->size_;
 * }
 * 
 * inline std::vector<int>
 * Graph::get_adjacency(std::size_t u)
 *   const
 * {
 *   if (u >= this->size_) return {};
 *   return this->adjacency_[u];
 * }
 * 
 * inline bool
 * Graph::add_edge(const std::size_t u,
 *                 const std::size_t v,
 *                 const bool dir)
 * {
 *   if (u >= this->size_ || v >= this->size_) {
 *     return false;
 *   }
 * 
 *   this->adjacency_[u].push_back(v);
 *   if (!dir) {
 *     this->adjacency_[v].push_back(u);
 *   }
 * 
 *   return true;
 * }
 */
/*}}}*/

class Model {
public:
  // Model(std::size_t num_of_crosses) : size_(num_of_crosses) {}

  Model(const std::vector<std::vector<int>> &cars,
        const std::vector<std::vector<int>> &roads,
        const std::vector<std::vector<int>> &crosses,
        const std::vector<std::vector<int>> &preset_cars);

  ~Model() {}

  // TODO: not finish yet.
  void initIndex();

  // TODO: not in consideration yet.
  // void update_roads_info(InfoPass &inps);

  // TODO: wait to qualify.
  Feedback dijkstra(StartEndInfo &start_end,
                    std::function<bool (const NodeInfo&, const NodeInfo&)> cmp,
                    std::function<int (StartEndInfo&, NodeInfo&)> cost);

  // XXX: require to design.  rate = start_time ^ 2 / all_car_require_time ?
  double time_rate(const int start_time, const int all_car_require_time);

  // TODO: wait to qualify.
  void increase_volumn(std::vector<int> &nodes);
  // TODO: wait to qulify.
  std::vector<int> transform_path(std::vector<int> &nodes);

private:
  Model() = default;
  int size_;

  std::vector<RawCar>      raw_cars_;
  std::vector<RawRoad>    raw_roads_;
  std::vector<RawCross> raw_crosses_;

  // XXX: Deprecated.
  // std::map<std::pair<int, int>, RawRoad*> uv_to_roads_;
  // std::map<std::pair<int, int>, InfoRoad> roads_;
  // std::map<std::pair<int, int>, int> cross_index_to_road_id_;
  // std::map<int, std::vector<NodeInfo>> adjacency_;
  // std::vector<int> counter_;

  // TODO: wait to qualify.
  std::map<int, int> cross_id_to_index_;

  // TODO: wait to qualify.
  std::map<std::pair<int, int>, RoadInfo> cross_index_to_road_info_;

  // TODO: wait to qualify.
  std::vector<std::vector<int>> adjacency_;

  // TODO: wait to qualify.
  std::vector<NodeInfo> node_info_;
};

inline
Model::Model(const std::vector<std::vector<int>> &cars,
             const std::vector<std::vector<int>> &roads,
             const std::vector<std::vector<int>> &crosses,
             const std::vector<std::vector<int>> &preset_cars)
{
  // TODO: obtain information and transform into raw_***s_.
}

inline void
Model::increase_volumn(std::vector<int> &nodes)
{
  for (auto n : nodes) {
    ++(this->node_info_[n].volumn);
  }
  return;
}

inline std::vector<int>
Model::transform_path(std::vector<int> &nodes) {
  std::vector<int> road_path;
  auto sz = nodes.size();
  for (auto i = 1; i < sz; ++i) {
    road_path.push_back(this->cross_index_to_road_info_[{ nodes[i - 1], nodes[i] }].id);
  }
  return road_path;
}

#endif // ifndef _MODEL_HPP_
