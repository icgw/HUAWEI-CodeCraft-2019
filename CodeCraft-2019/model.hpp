/*
 * model.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _MODEL_HPP_
#define _MODEL_HPP_

#include <iostream>

#include <map> 
#include <vector>
#include <unordered_map>
#include <functional> // std::function
#include <utility>    // std::pair

#include <ctime>      // std::time
#include <cstdlib>    // std::rand, std::srand

#include "io.hpp"

/*{{{ DEFINE MACRO */
#define   CAR_ID                  0
#define   CAR_FROM                1
#define   CAR_TO                  2
#define   CAR_SPEED               3
#define   CAR_PLAN_TIME           4
#define   CAR_PRIORITY            5
#define   CAR_PRESET              6
#define   CAR_SIZE                7

#define   ROAD_ID                 0
#define   ROAD_LEN                1
#define   ROAD_SPEED              2
#define   ROAD_CHANNEL            3
#define   ROAD_FROM               4
#define   ROAD_TO                 5
#define   ROAD_IS_DUPLEX          6
#define   ROAD_SIZE               7

#define   CROSS_ID                0
#define   CROSS_UP                1
#define   CROSS_RIGHT             2
#define   CROSS_DOWN              3
#define   CROSS_LEFT              4
#define   CROSS_SIZE              5

#define   PRESET_CAR_ID           0
#define   PRESET_CAR_START_TIME   1
#define   PRESET_CAR_ROAD_START   2
/*}}}*/

/*{{{ RawCar, RawRoad, RawCross, RawPresetCar. (up to the input data) */
struct RawCar {
  RawCar(int i, int f, int t, int s, int pt, int prr, int prs)
    : id(i), from(f), to(t), speed(s), plan_time(pt), priority(prr), preset(prs) {}
  int id, from, to, speed, plan_time, priority, preset;
};

struct RawRoad {
  RawRoad(int i, int l, int s, int c, int f, int t, int b)
    : id(i), len(l), speed(s), channel(c), from(f), to(t), is_duplex(b) {}
  int id, len, speed, channel, from, to, is_duplex;
};

struct RawCross {
  RawCross(int i, int up, int right, int down, int left)
    : id(i), r1(up), r2(right), r3(down), r4(left) {}
  int id, r1, r2, r3, r4;
};

// FIXME: ?constructor
struct RawPresetCar {
  RawPresetCar(int i, int s, std::vector<int> v)
    : id(i), start_time(s), road_path(v) {}
  int id, start_time;
  std::vector<int> road_path;
};
/*}}}*/

/*{{{ struct: Feedback, StartEndInfo, NodeInfo, RoadInfo */
/*
 * FIXME: may add more detail information and constructor. 
 *        DO NOT modify current symbol.
 */
struct Feedback {
  // int start_time, sugeest_start_time;
  std::vector<int> t_path;
  int cost_time;
};

struct StartEndInfo {
  StartEndInfo(int i, int st, int f, int t, int sp, int p, int b)
    : id(i)
    , start_time(st)
    , from_index(f)
    , to_index(t)
    , speed(sp)
    , priority(p)
    , is_preset(b) {}
  int id, start_time, from_index, to_index, speed, priority, is_preset;

  // NOTE: after initiating and compute_hotspot.
  int estimate_cost_time;
  std::vector<int> cross_index_seq;

  // TODO:
  int hot = 0;
};

// FIXME: considering...
struct NodeInfo {
  NodeInfo()
    : cost_time(0)
    , volumn(0)
    , in_degree(0)
    , out_degree(0)
    , hotspot(0) {}
  int index, cost_time, volumn;
  int in_degree, out_degree;
  int hotspot;
};

// FIXME: ??
struct RoadInfo {
  int id, len, speed, channel;
};
/*}}}*/

class Model {
public:
  Model(const std::string &car_path,
        const std::string &road_path,
        const std::string &cross_path,
        const std::string &preset_path,
        const std::string &answer_path);

  ~Model() {}

  // NOTE: after constructing.. map original id --> this model index.
  void initIndex();

  // NOTE: based on dijkstra algorithm.
  Feedback dijkstra(StartEndInfo &start_end,
                    std::function<bool (const NodeInfo&, const NodeInfo&)> cmp,
                    std::function<int (const StartEndInfo&, const RoadInfo&, const NodeInfo&)> cost);

  // XXX: require to design.  rate = start_time ^ 2 / all_car_require_time ?
  double time_rate(const int start_time, const int all_car_require_time);

  // TODO: wait to qualify.
  void increase_volumn(std::vector<int> &nodes);

  // NOTE: transform node id sequence to original road id sequence.
  std::vector<int> transform_path(std::vector<int> &nodes);

  // TODO: before running, probe and find some useful information and give it next.
  void probe();

  // TODO:
  void reorder_cars();

  // TODO: run model and store the answers.
  void run();

  // XXX: @deprecated
  //   -- IN: latest_time_
  void make_logistics_like(std::vector<int> &time_sequences);
  void make_logistics_like();

  // NOTE: output the answers stores in `this->answers_` (type: vector<vector<int>>).
  //   -- IN: output_path_, answers_
  void output_answers();

private:
  Model() = default;
  void transform_raw_data(const std::vector<std::vector<int>> &cars,
                          const std::vector<std::vector<int>> &roads,
                          const std::vector<std::vector<int>> &crosses,
                          const std::vector<std::vector<int>> &preset_cars);

  // NOTE: transform src_id, road_path_id, tgt_id --> node index sequence.
  std::vector<int> transform_original_path_to_cross_index(const int from_id, const std::vector<int> &roads, const int to_id);

  // NOTE: the number of crosses.
  int size_;

  // NOTE: the raw data of the model NOT input data.
  std::vector<RawCar>       raw_cars_;
  std::vector<RawRoad>      raw_roads_;
  std::vector<RawCross>     raw_crosses_;
  std::vector<RawPresetCar> raw_preset_cars_;
  /************************************************/

  // NOTE: extracted info. from raw data after calling `initIndex()`.
  std::map<int, int>                      cross_id_to_index_;
  std::map<std::pair<int, int>, RoadInfo> cross_index_to_road_info_;
  std::vector<std::vector<int>>           adjacency_;
  std::vector<NodeInfo>                   node_info_;
  std::vector<StartEndInfo>               cars_to_run_;
  std::map<int, std::pair<int, int>>      road_id_to_cross_index_;
  std::unordered_map<int, int>            preset_car_id_to_index_;
  std::map<int, std::map<int, int>>       from_road_id_to_to_id_;
  /****************************************************************/

  // NOTE:
  //   -- IN: adjacency_
  //   -- EFFECT: node_info_ will be modified.
  //              store the in-degree and out-degree for each node.
  void record_node_degree();

  // NOTE: parameter of thie model.
  void   default_parameter();
  bool   (*priority_cmp) (const NodeInfo&, const NodeInfo&);
  int    (*cost_func)    (const StartEndInfo&, const RoadInfo&, const NodeInfo&);
  int    latest_time_;
  int    start_time_;

  // TODO: random generator function
  int    (*random_call)(int i);

  // TODO: first schedule rate recommend 0.3
  double first_schedule_rate_;
  double first_schedule_time_rate_;

  // XXX: @deprecated
  double mid_point_;
  double lower_hotspot_cut_;       // recommend < 0.3 (>0)
  double upper_hotspot_cut_;       // recommend > 0.7 (<1)
  /*****************************************************************************/

  // NOTE: compute the time cost.
  int compute_estimate_cost(const int speed, const std::vector<int> &cross_idx);

  // NOTE: compute hot spot and record estimate time for each car.
  //   -- EFFECT: hotest_spot_cross_index_, cars_to_run_.estimate_cost_time, node_info_.hotspot.
  void compute_hotspot();

  // NOTE: compute passby cars for each cross id.
  //   -- IN: cars_to_run_;
  //      OUT: cross_index_to_passby_cars_.
  void compute_passby_cars();

  // TODO: must after compute_passby_cars()
  //   -- IN: hotest_spot_cross_index_, cars_to_run_
  //   -- compute each cars hot. sum of all cross index passby cars.
  void compute_cars_hot();

  // XXX: @deprecated
  // int lower_bound_hotspot_;
  // int upper_bound_hotspot_;

  // XXX: the most frequent pass-by cross index.
  // int hotest_spot_cross_index_;

  // NOTE: cross idx -> { car_index, ... }
  std::vector<std::vector<int>> cross_index_to_passby_cars_;

  // NOTE: set the output path, and store answer in this class.
  std::string                             output_path_;
  std::vector<std::vector<int>>           answers_;
  /***********************************************************/
};

inline
Model::Model(const std::string &car_path,
             const std::string &road_path,
             const std::string &cross_path,
             const std::string &preset_path,
             const std::string &answer_path)
{
  // XXX: 
  this->default_parameter();

  std::vector<std::vector<int>> cars, roads, crosses, preset_cars;
  read_from_file(car_path, CAR_SIZE, cars);
  read_from_file(road_path, ROAD_SIZE, roads);
  read_from_file(cross_path, CROSS_SIZE, crosses);

  read_from_file(preset_path, preset_cars);

  // XXX: process preset_cars;
  this->transform_raw_data(cars, roads, crosses, preset_cars);

  this->initIndex();

  this->record_node_degree();

  this->output_path_ = answer_path;
}

// NOTE: set the default parameters of the model here.
inline void
Model::default_parameter()
{
  this->start_time_               = 600;
  this->latest_time_              = 2600;
  this->first_schedule_rate_      = 0.3;
  this->first_schedule_time_rate_ = 0.5;

  // FIXME: not use?
  this->mid_point_ = 0.3;

  /*
   * double lower_hotspot_cut_ = 0.3;
   * double upper_hotspot_cut_ = 0.7 ;
   */

  // `true` means first element `a` is weaker priority order than second element `b`,
  // otherwise first element `a` is more priority than second element `b`.
  this->priority_cmp = [](const NodeInfo &a, const NodeInfo &b) -> bool {
    return a.cost_time > b.cost_time ||
          (a.cost_time <= b.cost_time && a.hotspot > b.hotspot);
  };

  // cost function to computer src to current node. (must return > 0)
  this->cost_func = [](const StartEndInfo &st, const RoadInfo &r, const NodeInfo &n) -> int {
    // int len   = r.len;
    // int limit = r.speed;
    // int min_v = std::min(st.speed, limit);
    // return (len + min_v - 1) / min_v;
    // return (int) (n.hotspot * (rand() / (double) RAND_MAX));
    return n.hotspot + n.volumn;
  };

  // XXX: set time seed?
  std::srand(std::time(0));
  this->random_call = [](int i) -> int { return std::rand() % i; };

  return;
}

inline
std::vector<int>
Model::transform_original_path_to_cross_index(const int from_id,
                                              const std::vector<int> &roads,
                                              const int to_id)
{
  std::vector<int> ret;
  int from = from_id, to;
  ret.push_back(this->cross_id_to_index_[from]);
  for (auto rd : roads) {
    to = this->from_road_id_to_to_id_[from][rd];
    ret.push_back(this->cross_id_to_index_[to]);
    from = to;
  }
  // FIXME: assert end_to == to_id;
  return ret;
}

inline int
Model::compute_estimate_cost(const int speed,
                             const std::vector<int> &cross_idx)
{
  int ret = 0, len, limit, min_v;
  int sz = cross_idx.size();
  for (int i = 1; i < sz; ++i) {
    RoadInfo rinfo = this->cross_index_to_road_info_[{ cross_idx[i - 1], cross_idx[i] }];
    len   = rinfo.len;
    limit = rinfo.speed;
    min_v = std::min(speed, limit);
    ret  += (int) ((len + min_v - 1) / min_v);
  }
  return ret;
}

// XXX: emm...
inline void
Model::compute_passby_cars()
{
  int sz = this->cars_to_run_.size();
  for (auto i = 0; i < sz; ++i) {
    for (auto &idx : this->cars_to_run_[i].cross_index_seq) {
      this->cross_index_to_passby_cars_[idx].push_back(i);
    }
  }
  return;
}

// XXX: emmm...
inline void
Model::compute_cars_hot()
{
  for (auto &st : this->cars_to_run_) {
    for (auto &idx : st.cross_index_seq) {
      // st.hot += this->cross_index_to_passby_cars_[idx].size();
      st.hot += this->node_info_[idx].volumn;
    }
  }
  return;
}

// FIXME: not useful?
inline void
Model::record_node_degree()
{
  for (auto i = 0; i < this->size_; ++i) {
    this->node_info_[i].out_degree = this->adjacency_[i].size();
    for (auto n : this->adjacency_[i]) {
      ++(this->node_info_[n].in_degree);
    }
  }

  return;
}

inline void
Model::transform_raw_data(const std::vector<std::vector<int>> &cars,
                          const std::vector<std::vector<int>> &roads,
                          const std::vector<std::vector<int>> &crosses,
                          const std::vector<std::vector<int>> &preset_cars)
{
  this->raw_cars_.reserve(cars.size());
  this->raw_roads_.reserve(roads.size());
  this->raw_crosses_.reserve(crosses.size());

  // TODO: obtain information and transform into raw_***s_.
  for (auto &v : cars) {
    this->raw_cars_.push_back(
        RawCar(v[CAR_ID], v[CAR_FROM], v[CAR_TO], v[CAR_SPEED], v[CAR_PLAN_TIME],
               v[CAR_PRIORITY], v[CAR_PRESET])
        );
  }

  for (auto &v : roads) {
    this->raw_roads_.push_back(
        RawRoad(v[ROAD_ID], v[ROAD_LEN], v[ROAD_SPEED], v[ROAD_CHANNEL],
                v[ROAD_FROM], v[ROAD_TO], v[ROAD_IS_DUPLEX])
        );
  }

  for (auto &v : crosses) {
    this->raw_crosses_.push_back(
        RawCross(v[CROSS_ID], v[CROSS_UP], v[CROSS_RIGHT], v[CROSS_DOWN], v[CROSS_LEFT])
        );
  }

  for (auto &v : preset_cars) {
    this->raw_preset_cars_.push_back(
        RawPresetCar(v[PRESET_CAR_ID], v[PRESET_CAR_START_TIME],
                     std::vector<int>(v.begin() + PRESET_CAR_ROAD_START, v.end()))
        );
  }

  return;
}

// FIXME: not useful?
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
  int sz = nodes.size();
  for (auto i = 1; i < sz; ++i) {
    road_path.push_back(this->cross_index_to_road_info_[{ nodes[i - 1], nodes[i] }].id);
  }
  return road_path;
}

inline void
Model::output_answers()
{
  write_to_file(this->output_path_, this->answers_);
  return;
}

#endif // ifndef _MODEL_HPP_
