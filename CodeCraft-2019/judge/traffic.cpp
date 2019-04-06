/*
 * traffic.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <algorithm>  // std::max
#include "traffic.hpp"

/*{{{ Identity::get_id() */
inline int
Identity::get_id()
  const
{
  return this->id_;
}
/*}}}*/

/*{{{ StartEnd::get_speed(), get_from(), get_to() */
inline int
StartEnd::get_speed()
  const
{
  return this->speed_;
}

inline int
StartEnd::get_from()
  const
{
  return this->from_;
}

inline int
StartEnd::get_to()
  const
{
  return this->to_;
}
/*}}}*/

/*{{{ Car::get_plan_time(), get_priority(), get_preset() */
inline int
Car::get_plan_time()
  const
{
  return this->plan_time_;
}

inline int
Car::get_priority()
  const
{
  return this->priority_;
}

inline int
Car::get_preset()
  const
{
  return this->preset_;
}
/*}}}*/

/*{{{ Road::get_length() */
inline int
Road::get_length()
  const
{
  return this->length_;
}
/*}}}*/

void
Cross::init(std::unordered_map<int, RoadOnline*> road_id_to_roadonline)
{
  for (auto id : this->roads_id_) {
    if (id == -1) {
      continue;
    }
    this->roads_online_.push_back(road_id_to_roadonline[id]);
  }
  return;
}

/*{{{ RunningCar::get_start_time(), get_current_road_pos(), get_state(), set_current_road_pos(), set_state(), set_current_road_idx() */
inline int
RunningCar::get_start_time()
  const
{
  return this->start_time_;
}

inline int
RunningCar::get_current_road_pos()
  const
{
  return this->current_road_pos_;
}

inline int
RunningCar::get_current_road_channel()
  const
{
  return this->current_road_channel_;
}

inline State
RunningCar::get_state()
  const
{
  return this->state_;
}

inline void
RunningCar::set_current_road_pos(const int p)
{
  this->current_road_pos_ = p;
  return;
}

inline void
RunningCar::set_current_road_channel(const int channel)
{
  this->current_road_channel_ = channel;
  return;
}

inline void
RunningCar::set_state(const State s)
{
  this->state_ = s;
  return;
}

inline void
RunningCar::set_current_road_idx(const std::size_t idx)
{
  this->idx_of_current_road_ = idx;
  return;
}
/*}}}*/

void
RunningCar::init(std::vector<RoadOnline*> &p)
{
  this->path_.assign(p.begin(), p.end());
  this->state_                = WAIT;
  this->current_road_pos_     = 0;
  this->current_road_channel_ = -1;
  this->next_road_pos_        = 0;

  this->idx_of_current_road_  = -1;
}

bool
RunningCar::move_to_next_road() // for waiting car.
{
  if (FINISH == this->state_) {
    return true;
  }

  auto next_road_idx         = this->idx_of_current_road_ + 1;
  if (next_road_idx >= this->path_.size()) {
    return true;
  }

  auto next_road_start_cross = this->start_cross_id_sequence_[next_road_idx]->get_id();
  auto next_road = this->path_[next_road_idx];

  if (next_road->is_final_filled(next_road_start_cross)) {
    this->current_road_pos_ = this->path_[this->idx_of_current_road_]->get_length();
    this->state_ = FINAL;
    return true;
  }

  auto next_road_channel_and_pos = next_road->select_valid_channel(next_road_start_cross);
  auto next_channel = next_road_channel_and_pos.first;
  auto next_pos     = next_road_channel_and_pos.second;

  if (next_channel != -1) {
    ++this->idx_of_current_road_;

    this->current_road_channel_ = next_channel;
    this->current_road_pos_     = std::min(this->next_road_pos_, next_pos);
    this->next_road_pos_        = 0;
    this->state_                = FINAL;
    return true;
  }

  return false;
}

void
RunningCar::drive(const int speed)
{
  int current_road_len = this->path_[this->idx_of_current_road_]->get_length();
  if (this->current_road_pos_ + speed <= current_road_len) {
    this->current_road_pos_ += speed;
    this->state_             = FINAL;
    return;
  }

  if (this->path_.size() - 1 <= this->idx_of_current_road_) {
    this->current_road_pos_ += speed;
    this->state_             = FINISH;
    return;
  }

  int s1 = current_road_len - this->current_road_pos_;
  RoadOnline *next_road = this->path_[this->idx_of_current_road_ + 1];
  int v2 = std::min(this->speed_, next_road->get_speed());

  this->current_road_pos_ = current_road_len;
  this->next_road_pos_ = std::max(0, v2 - s1);
  this->state_ = ((this->next_road_pos_ == 0) ? FINAL : WAIT);

  return;
}

bool
compare_priority(RunningCar* const &c1,
                 RunningCar* const &c2)
{
  return c1->get_priority()  > c2->get_priority() ||
        (c1->get_priority() == c2->get_priority() && c1->get_start_time()  < c2->get_start_time()) ||
        (c1->get_priority() == c2->get_priority() && c1->get_start_time() == c2->get_start_time() && c1->get_id() < c2->get_id());
}

void
RoadInitCarList::create_car_sequence()
{
  dir_cars_.sort(compare_priority);
  if (1 == this->is_duplex_) {
    inv_cars_.sort(compare_priority);
  }
  return;
}

void
RoadOnline::drive_just_current_road(const int channel,
                                    const int start_cross_id)
{
  if (start_cross_id == this->from_) {
    this->drive_just_current_road(channel, this->dir_on_running_cars_);
  }
  else if (start_cross_id == this->to_) {
    this->drive_just_current_road(channel, this->inv_on_running_cars_);
  }
  return;
}

void
RoadOnline::drive_just_current_road()
{
  for (auto i = 0; i < this->channel_; ++i) {
    drive_just_current_road(i, this->from_);
  }

  if (1 == this->is_duplex_) {
    for (auto i = 0; i < this->channel_; ++i) {
      drive_just_current_road(i, this->to_);
    }
  }
  return;
}

void
RoadOnline::drive_just_current_road(const int channel,
                                    std::vector<std::vector<RunningCar*>> &cars)
{
  if (channel < 0 || channel >= cars.size()) {
    return;
  }

  int v = 0, prev_pos = this->length_ + 1;
  State prev_state = WAIT;
  for (auto c : cars[channel]) {
    v = std::min(c->get_speed(), this->get_speed());
    if (v >= prev_pos - c->get_current_road_pos()) {
      if (prev_state == FINAL) {
        c->drive(prev_pos - c->get_current_road_pos() - 1);
      }
      c->set_state(prev_state);
    } else {
      c->drive(v);
      c->set_state(FINAL);
    }
    prev_pos   = c->get_current_road_pos();
    prev_state = c->get_state();
  }

  return;
}

bool
RoadOnline::run_to_road(RunningCar* c,
                        std::vector<std::vector<RunningCar*>> &running_cars)
{
  if (nullptr == c) return false;

  int v = std::min(c->get_speed(), this->get_speed());

  int sz = running_cars.size();
  for (auto i = 0; i < sz; ++i) {
    if (running_cars[i].back()->get_current_road_pos() <= 1) {
      continue;
    }

    if (running_cars[i].back()->get_current_road_pos() <= v && running_cars[i].back()->get_state() == WAIT) {
      continue;
    }

    v = std::min(v, running_cars[i].back()->get_current_road_pos() - 1);

    c->set_current_road_pos(v);
    c->set_state(running_cars[i].back()->get_state());
    c->set_current_road_idx(0);
    c->set_current_road_channel(i);
    running_cars[i].push_back(c);

    return true;
  }

  return false;
}

void
RoadOnline::run_car_in_init_list(const int current_time,
                                 std::list<RunningCar*> &init_list,
                                 std::vector<std::vector<RunningCar*>> &running_cars)
{
  for (auto it = init_list.begin(); it != init_list.end(); ) {
    if (current_time < (*it)->get_start_time()) {
      ++it;
      continue;
    }

    if (run_to_road(*it, running_cars)) {
      it = init_list.erase(it);
    } else {
      ++it;
    }
  }
  return;
}

void
RoadOnline::run_car_in_init_list(int current_time)
{
  run_car_in_init_list(current_time, this->dir_cars_, this->dir_on_running_cars_);
  if (1 == this->is_duplex_) {
    run_car_in_init_list(current_time, this->inv_cars_, this->inv_on_running_cars_);
  }
  return;
}

bool
RoadOnline::is_final_filled(const int start_cross_id)
{
  if (start_cross_id == this->from_) {
    for (auto &channel : this->dir_on_running_cars_) {
      if (channel.size() <= 0) {
        return false;
      }
      if (channel.back()->get_state() != FINAL || channel.back()->get_current_road_pos() > 1) {
        return false;
      }
    }
  }
  else if (1 == this->is_duplex_ && start_cross_id == this->to_) {
    for (auto &channel : this->inv_on_running_cars_) {
      if (channel.size() <= 0) {
        return false;
      }
      if (channel.back()->get_state() != FINAL || channel.back()->get_current_road_pos() > 1) {
        return false;
      }
    }
  }
  return true;
}

std::pair<int, int>
RoadOnline::select_valid_channel(const int start_cross_id)
{
  if (start_cross_id == this->from_) {
    auto sz = this->dir_on_running_cars_.size();
    for (auto i = 0; i < sz; ++i) {
      if (this->dir_on_running_cars_[i].size() == 0) {
        return { i, this->length_ };
      }
      else if (this->dir_on_running_cars_[i].back()->get_current_road_pos() > 1 &&
               this->dir_on_running_cars_[i].back()->get_state() == FINAL) {
        return { i, this->dir_on_running_cars_[i].back()->get_current_road_pos() - 1 };
      }
    }
  }
  else if (1 == this->is_duplex_ && start_cross_id == this->to_) {
    auto sz = this->inv_on_running_cars_.size();
    for (auto i = 0; i < sz; ++i) {
      if (this->inv_on_running_cars_[i].size() == 0) {
        return { i, this->length_ };
      }
      else if (this->inv_on_running_cars_[i].back()->get_current_road_pos() > 1 &&
               this->inv_on_running_cars_[i].back()->get_state() == FINAL) {
        return { i, this->inv_on_running_cars_[i].back()->get_current_road_pos() - 1 };
      }
    }
  }

  return { -1, 0 };
}
