/*
 * model.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */
// #include <iostream>

#include <limits>    // std::numeric_limits<double>::infinity()
#include <algorithm> // std::reverse
#include <queue>     // std::priority_queue

#include <cmath>     // std::pow

#include "model.hpp"

/*{{{ initialization of model index to orginal data index */
void
Model::initIndex()
{
  // NOTE: call the method after model initialization.
  auto sz = this->raw_crosses_.size();

  this->size_ = sz;
  this->node_info_.resize(sz);
  this->adjacency_.resize(sz);

  // NOTE: create cross_id_to_index.
  for (auto i = 0; i < sz; ++i) {
    this->cross_id_to_index_[this->raw_crosses_[i].id] = i;
    this->node_info_[i].index  = i;
  }

  // NOTE: create cross_index_to_road_info_,
  //        adjacency_, road_id_to_cross_index_.
  //   -- extract raw_roads information we need.
  sz = this->raw_roads_.size();
  int from_idx, to_idx;
  for (auto i = 0; i < sz; ++i) {
    from_idx = this->cross_id_to_index_[this->raw_roads_[i].from];
    to_idx   = this->cross_id_to_index_[this->raw_roads_[i].to];

    // NOTE: road_id --> (from_cross_index, to_cross_index)
    this->road_id_to_cross_index_[this->raw_roads_[i].id] = std::make_pair(from_idx, to_idx);

    this->adjacency_[from_idx].push_back(to_idx);
    this->cross_index_to_road_info_[{ from_idx, to_idx }].id      = this->raw_roads_[i].id;
    this->cross_index_to_road_info_[{ from_idx, to_idx }].len     = this->raw_roads_[i].len;
    this->cross_index_to_road_info_[{ from_idx, to_idx }].speed   = this->raw_roads_[i].speed;
    this->cross_index_to_road_info_[{ from_idx, to_idx }].channel = this->raw_roads_[i].channel;
    if (0 != this->raw_roads_[i].is_duplex) {
      this->cross_index_to_road_info_[{ to_idx, from_idx }].id      = this->raw_roads_[i].id;
      this->cross_index_to_road_info_[{ to_idx, from_idx }].len     = this->raw_roads_[i].len;
      this->cross_index_to_road_info_[{ to_idx, from_idx }].speed   = this->raw_roads_[i].speed;
      this->cross_index_to_road_info_[{ to_idx, from_idx }].channel = this->raw_roads_[i].channel;
      this->adjacency_[to_idx].push_back(from_idx);
    }
  }

  // FIXME: may require to consider preset car...
  sz = this->raw_cars_.size();
  for (auto i = 0; i < sz; ++i) {
    if (this->raw_cars_[i].preset != 0) {
      continue;
    }
    from_idx = this->cross_id_to_index_[this->raw_cars_[i].from];
    to_idx = this->cross_id_to_index_[this->raw_cars_[i].to];
    this->cars_to_run_.push_back(StartEndInfo(this->raw_cars_[i].id,
                                              this->raw_cars_[i].plan_time,
                                              from_idx,
                                              to_idx,
                                              this->raw_cars_[i].speed));
  }

  return;
}
/*}}}*/

Feedback
Model::dijkstra(StartEndInfo &start_end,
                std::function<bool (const NodeInfo&, const NodeInfo&)> cmp,
                std::function<int (const StartEndInfo&, const NodeInfo&)> cost)
{
  Feedback fb; fb.t_path.clear();
  std::priority_queue<NodeInfo,
                      std::vector<NodeInfo>,
                      decltype(cmp)> pq(cmp);

  std::vector<int> trace (this->size_);
  std::vector<double> dist (this->size_, std::numeric_limits<double>::infinity());

  NodeInfo src;
  src.cost_time    = start_end.start_time;
  src.index        = start_end.from_index;
  src.volumn       = this->node_info_[src.index].volumn;
  dist[src.index]  = cost(start_end, src);
  trace[src.index] = -1;

  pq.push(src);
  int len, limit, min_v, v_cost_time, w;

  while (!pq.empty()) {
    NodeInfo u = pq.top();
    pq.pop();
    for (auto &v_idx : this->adjacency_[u.index]) {
      len          = this->cross_index_to_road_info_[{ u.index, v_idx }].len;
      limit        = this->cross_index_to_road_info_[{ u.index, v_idx }].speed;
      min_v        = std::min(start_end.speed, limit);
      v_cost_time  = (len + min_v - 1) / min_v;

      this->node_info_[v_idx].cost_time = v_cost_time + u.cost_time;
      w = cost(start_end, this->node_info_[v_idx]);

      if (dist[v_idx] > dist[u.index] + w) {
        dist[v_idx]  = dist[u.index] + (double) w;
        trace[v_idx] = u.index;
        pq.push(this->node_info_[v_idx]);
      }
    }
  }

  int to = start_end.to_index;
  while (trace[to] != -1) {
    fb.t_path.push_back(to);
    to = trace[to];
  }

  fb.t_path.push_back(start_end.from_index);
  reverse(fb.t_path.begin(), fb.t_path.end());
  fb.cost_time = this->node_info_[start_end.to_index].cost_time;

  /*
   * std::vector<int> xpath = this->transform_path(fb.t_path);
   * for (auto x : xpath) {
   *   std::cout << x << ", ";
   * }
   * std::cout << std::endl;
   */

  return fb;
}

// FIXME: useful???
void
Model::make_logistics_like(std::vector<int> &time_sequences)
{
  auto sz    = time_sequences.size();
  double mid = (double) sz * this->mid_point;
  double L   = this->latest_time_;
  double k   = std::pow(L / time_sequences[0] + 1.0, 1.0 / mid);
  double e   = time_sequences[0];

  for (auto i = 1; i < sz; ++i) {
    e = L / ((L / e - 1) / k + 1);
    time_sequences[i] = std::max(time_sequences[i], (int) e);
  }

  return;
}

void
Model::run()
{
  for (auto st : this->cars_to_run_) {
    Feedback fb = this->dijkstra(st, this->priority_cmp, this->cost_func);
  }

  return;
}
