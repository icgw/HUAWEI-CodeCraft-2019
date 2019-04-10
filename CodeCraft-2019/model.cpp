/*
 * model.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <algorithm> // std::reverse
#include <queue>     // std::priority_queue

#include "model.hpp"

void
Model::initIndex()
{
  // XXX: after model initialization.
  auto sz = this->raw_crosses_.size();
  this->adjacency_.resize(sz);

  // TODO: create cross_id_to_index, wait to qualify.
  for (auto i = 0; i < sz; ++i) {
    this->cross_id_to_index_[this->raw_crosses_[i].id] = i;
  }

  // TODO: create cross_index_to_road_info_, adjacency_. wait to qualify.
  //   -- FINISH extract raw_roads information completely.
  sz = this->raw_roads_.size();
  int from_idx, to_idx;
  for (auto i = 0; i < sz; ++i) {
    from_idx = this->cross_id_to_index_[this->raw_roads_[i].from];
    to_idx   = this->cross_id_to_index_[this->raw_roads_[i].to];
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

  // TODO: car?

  return;
}

Feedback
Model::dijkstra(StartEndInfo &start_end,
                std::function<bool (const NodeInfo&, const NodeInfo&)> cmp,
                std::function<int (StartEndInfo&, NodeInfo&)> cost)
{
  Feedback fb;
  std::priority_queue<NodeInfo,
                      std::vector<NodeInfo>,
                      decltype(cmp)> pq(cmp);

  std::vector<int> trace (this->size_);
  // std::map<int, int> dist;
  std::vector<int> dist (this->size_);

  NodeInfo src;
  src.cost_time    = start_end.start_time;
  src.index        = start_end.from_index;
  src.volumn       = this->node_info_[src.index].volumn;
  dist[src.index]  = cost(start_end, src);
  trace[src.index] = -1;

  int len, limit, min_v, v_cost_time;
  while (!pq.empty()) {
    NodeInfo u = pq.top();
    pq.pop();
    for (auto &v_idx : this->adjacency_[u.index]) {
      auto v = this->node_info_[v_idx];
      int w  = cost(start_end, v);
      if (dist[v_idx] > dist[u.index] + w) {
        dist[v_idx]  = dist[u.index] + w;
        len          = this->cross_index_to_road_info_[{ u.index, v_idx }].len;
        limit        = this->cross_index_to_road_info_[{ u.index, v_idx }].speed;
        min_v        = std::min(start_end.speed, limit);
        v_cost_time  = (len + min_v - 1) / min_v;
        v.cost_time  = v_cost_time + u.cost_time;
        trace[v_idx] = u.index;

        pq.push(v);
      }
    }
  }

  int to = start_end.to_index;
  fb.t_path.push_back(to);
  while (trace[to] != -1) {
    fb.t_path.push_back(trace[to]);
  }
  fb.t_path.push_back(start_end.from_index);
  reverse(fb.t_path.begin(), fb.t_path.end());
  fb.cost_time = dist[start_end.to_index];

  return fb;
}
