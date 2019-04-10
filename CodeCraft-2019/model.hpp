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
#include <utility>    // std::pair

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

/*{{{ RawCar, RawRoad, RawCross.*/
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
/*}}}*/

struct Feedback {
  // int start_time, sugeest_start_time;
  std::vector<int> t_path;
  int cost_time;
};

struct StartEndInfo {
  StartEndInfo(int i, int st, int f, int t, int sp)
    : id(i), start_time(st), from_index(f), to_index(t), speed(sp) {}
  int id, start_time, from_index, to_index, speed;
};

struct NodeInfo {
  int index, cost_time, volumn;
};

struct RoadInfo {
  int id, len, speed, channel;
};

class Model {
public:
  Model(const std::string &car_path,
        const std::string &road_path,
        const std::string &cross_path,
        const std::string &preset_path,
        const std::string &answer_path);

  void transform_raw_data(const std::vector<std::vector<int>> &cars,
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
                    std::function<int (const StartEndInfo&, const NodeInfo&)> cost);

  // XXX: require to design.  rate = start_time ^ 2 / all_car_require_time ?
  double time_rate(const int start_time, const int all_car_require_time);

  // TODO: wait to qualify.
  void increase_volumn(std::vector<int> &nodes);
  // TODO: wait to qulify.
  std::vector<int> transform_path(std::vector<int> &nodes);

  void run();

private:
  Model() = default;
  int size_;

  std::vector<RawCar>      raw_cars_;
  std::vector<RawRoad>    raw_roads_;
  std::vector<RawCross> raw_crosses_;

  // TODO: wait to qualify.
  std::map<int, int> cross_id_to_index_;
  std::map<std::pair<int, int>, RoadInfo> cross_index_to_road_info_;
  std::vector<std::vector<int>> adjacency_;
  std::vector<NodeInfo> node_info_;
  std::vector<StartEndInfo> cars_to_run_;

  // XXX:
  std::string output_path_;
};

inline
Model::Model(const std::string &car_path,
             const std::string &road_path,
             const std::string &cross_path,
             const std::string &preset_path,
             const std::string &answer_path)
{
  std::vector<std::vector<int>> cars, roads, crosses, preset_cars;
  read_from_file(car_path, CAR_SIZE, cars);
  read_from_file(road_path, ROAD_SIZE, roads);
  read_from_file(cross_path, CROSS_SIZE, crosses);

  read_from_file(preset_path, preset_cars);

  this->transform_raw_data(cars, roads, crosses, preset_cars);

  this->initIndex();

  this->output_path_ = answer_path;
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

  // XXX: how to process preset?
  return;
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
