/*
 * model.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

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
  int sz = this->raw_crosses_.size();

  this->size_ = sz;
  this->node_info_.resize(sz);
  this->adjacency_.resize(sz);
  this->cross_index_to_passby_cars_.resize(sz);

  // NOTE: create cross_id_to_index.
  for (auto i = 0; i < sz; ++i) {
    this->cross_id_to_index_[this->raw_crosses_[i].id] = i;
    this->node_info_[i].index                          = i;
  }

  // NOTE: create cross_index_to_road_info_,
  //        adjacency_, road_id_to_cross_index_, from_road_id_to_to_id_.
  //   -- extract raw_roads information we need.
  sz = this->raw_roads_.size();
  int from, to, from_idx, to_idx;
  for (auto i = 0; i < sz; ++i) {
    from     = this->raw_roads_[i].from;
    to       = this->raw_roads_[i].to;
    this->from_road_id_to_to_id_[from][this->raw_roads_[i].id] = to;

    from_idx = this->cross_id_to_index_[from];
    to_idx   = this->cross_id_to_index_[to];

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
      this->from_road_id_to_to_id_[to][this->raw_roads_[i].id] = from;
    }
  }

  // NOTE: map preset car id to index.
  sz = this->raw_preset_cars_.size();
  for (auto i = 0; i < sz; ++i) {
    this->preset_car_id_to_index_[this->raw_preset_cars_[i].id] = i;
  }

  // NOTE: save preset car's path.
  sz = this->raw_cars_.size();
  for (auto i = 0; i < sz; ++i) {
    from_idx = this->cross_id_to_index_[this->raw_cars_[i].from];
    to_idx   = this->cross_id_to_index_[this->raw_cars_[i].to];

    StartEndInfo start_end(this->raw_cars_[i].id,
                           this->raw_cars_[i].plan_time,
                           from_idx,
                           to_idx,
                           this->raw_cars_[i].speed,
                           this->raw_cars_[i].priority,
                           this->raw_cars_[i].preset);

    if (this->raw_cars_[i].preset != 0) {
      int idx = this->preset_car_id_to_index_[this->raw_cars_[i].id];
      std::vector<int> cross_seq = this->transform_original_path_to_cross_index(
          this->raw_cars_[i].from,
          this->raw_preset_cars_[idx].road_path,
          this->raw_cars_[i].to);

      start_end.cross_index_seq.assign(cross_seq.begin(), cross_seq.end());
    }
    this->cars_to_run_.push_back(start_end);
  }

  return;
}
/*}}}*/

/*{{{ dijkstra algorihtm(st, cmp, cost) */
Feedback
Model::dijkstra(StartEndInfo &start_end,
                std::function<bool (const NodeInfo&, const NodeInfo&)> cmp,
                std::function<int (const StartEndInfo&, const RoadInfo&, const NodeInfo&)> cost)
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

  RoadInfo tmp;
  tmp.id = -1; tmp.len = 0; tmp.speed = 0x3f3f3f3f; tmp.channel = 0x3f3f3f3f;
  dist[src.index]  = cost(start_end, tmp, src);
  trace[src.index] = -1;

  pq.push(src);

  int len, limit, min_v, v_cost_time;
  double w;

  while (!pq.empty()) {
    NodeInfo u = pq.top();
    pq.pop();
    for (auto &v_idx : this->adjacency_[u.index]) {
      RoadInfo r  = this->cross_index_to_road_info_[{ u.index, v_idx }];
      len         = r.len;
      limit       = r.speed;
      min_v       = std::min(start_end.speed, limit);
      v_cost_time = (len + min_v - 1) / min_v;

      this->node_info_[v_idx].cost_time = v_cost_time + u.cost_time;
      w = (double) cost(start_end, r, this->node_info_[v_idx]);

      if (dist[v_idx] > dist[u.index] + w) {
        dist[v_idx]  = dist[u.index] + w;
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
  fb.cost_time = this->node_info_[start_end.to_index].cost_time - start_end.start_time;

  return fb;
}
/*}}}*/

/*{{{ make_logistics_like(vector<int>): make time sequence logistics like */
// FIXME: @deprecated.
void
Model::make_logistics_like(std::vector<int> &time_sequences)
{
  int sz     = time_sequences.size();
  double mid = (double) sz * this->mid_point_;
  double L   = this->latest_time_;
  double k   = std::pow(L / time_sequences[0] + 1.0, 1.0 / mid);
  double e   = time_sequences[0];

  for (auto i = 1; i < sz; ++i) {
    e = L / ((L / e - 1) / k + 1);
    time_sequences[i] = std::max(time_sequences[i], (int) e);
  }

  return;
}

// FIXME: @deprecated.
void
Model::make_logistics_like()
{
  int sz = this->cars_to_run_.size();
  double mid = (double) sz * this->mid_point_;
  double L = this->latest_time_;
  double k = std::pow(L / this->cars_to_run_[0].start_time + 1.0, 1.0 / mid);
  double e = this->cars_to_run_[0].start_time;
  for (auto i = 1; i < sz; ++i) {
    e = L / ((L / e - 1) / k + 1);
    this->cars_to_run_[i].start_time = std::max(this->cars_to_run_[i].start_time, (int) e);
  }
  return;
}
/*}}}*/

void
Model::probe()
{
  this->compute_hotspot();

  // std::random_shuffle(this->cars_to_run_.begin(), this->cars_to_run_.end(),
  //                     this->random_call);
  return;
}

void
Model::run()
{
  this->probe();

  for (auto &st : this->cars_to_run_) {
    if (st.is_preset != 0) {
      // TODO: ?
      for (auto idx : st.cross_index_seq) {
        ++(this->node_info_[idx].volumn);
      }
      continue;
    }

    Feedback fb = this->dijkstra(st, this->priority_cmp, this->cost_func);
    st.cross_index_seq.assign(fb.t_path.begin(), fb.t_path.end());
    for (auto idx : fb.t_path) {
      ++(this->node_info_[idx].volumn);
    }
  }

  this->compute_passby_cars();
  this->compute_cars_hot();

  // TODO: compute start time twice.
  std::sort(this->cars_to_run_.begin(), this->cars_to_run_.end(),
      [](const StartEndInfo &a, const StartEndInfo &b) -> bool {
        return a.hot > b.hot;
      });

  int total = this->cars_to_run_.size();

  //  -- step 1: for part hot car.
  double start_t = (double) this->start_time_;
  int part       = (int) total * this->first_schedule_rate_;
  double step1   = (double) this->latest_time_ * this->first_schedule_time_rate_ / (double) part; // key.
  for (int i = 0; i < part; ++i) {
    this->cars_to_run_[i].start_time = std::max(this->cars_to_run_[i].start_time, (int)start_t);
    start_t += step1;
  }

  //  -- step 2: for remain car contail cold car.
  // std::random_shuffle(this->cars_to_run_.begin(), this->cars_to_run_.end(),
  //                     this->random_call);
  // NOTICE?
  start_t = 1;
  std::sort(this->cars_to_run_.begin(), this->cars_to_run_.end(),
      [](const StartEndInfo &a, const StartEndInfo &b) -> bool {
        return a.priority > b.priority ||
               (a.priority == b.priority && a.estimate_cost_time < b.estimate_cost_time);
      });
  double step2 = (double) this->latest_time_ / (double) total;
  for (int i = 0; i < total; ++i) {
    if (this->cars_to_run_[i].is_preset != 1) {
      this->cars_to_run_[i].start_time = std::max(this->cars_to_run_[i].start_time, (int) start_t);
    }
    start_t += step2;
  }

  for (auto &st : this->cars_to_run_) {
    if (st.is_preset == 1) {
      continue;
    }
    std::vector<int> tmp;
    tmp.push_back(st.id);
    tmp.push_back(st.start_time);
    std::vector<int> road_path = this->transform_path(st.cross_index_seq);

    tmp.insert(tmp.end(), road_path.begin(), road_path.end());
    this->answers_.push_back(tmp);
  }

  return;
}

// compute hotspot with classical shortest path.
void
Model::compute_hotspot()
{
  static auto cmp = [](const NodeInfo &a, const NodeInfo &b) -> bool {
    return a.cost_time > b.cost_time;
  };

  static auto cost_func = [](const StartEndInfo &st, const RoadInfo &r, const NodeInfo &n) -> int {
    int len   = r.len;
    int limit = r.speed;
    int min_v = std::min(st.speed, limit);
    return (len + min_v - 1) / min_v;
  };

  for (auto &st : this->cars_to_run_) {
    if (st.is_preset != 0) {
      // NOTE: for preset car or non-preset car, compute hotspot separately.
      st.estimate_cost_time = this->compute_estimate_cost(st.speed, st.cross_index_seq);
      for (auto idx : st.cross_index_seq) {
        ++(this->node_info_[idx].hotspot);
      }
    } else {
      Feedback fb = this->dijkstra(st, cmp, cost_func);
      st.estimate_cost_time = fb.cost_time;
      for (auto idx : fb.t_path) {
        ++(this->node_info_[idx].hotspot);
      }
    }
  }

  return;
}

// FIXME: not useful?
void
Model::reorder_cars()
{
  std::sort(this->cars_to_run_.begin(), this->cars_to_run_.end(),
      [](const StartEndInfo &a, const StartEndInfo &b) -> bool {
        return (a.estimate_cost_time > b.estimate_cost_time) ||
               (a.estimate_cost_time == b.estimate_cost_time && a.priority > b.priority) ||
               (a.estimate_cost_time == b.estimate_cost_time && a.priority == b.priority && a.start_time < b.start_time) ||
               (a.estimate_cost_time == b.estimate_cost_time && a.priority == b.priority && a.start_time == b.start_time && a.id < b.id);
      });
  return;
}
