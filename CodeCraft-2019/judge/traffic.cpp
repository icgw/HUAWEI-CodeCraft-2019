/*
 * traffic.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <algorithm>  // std::max
#include "traffic.hpp"

/*{{{ Identity::get_id() */
int
Identity::get_id()
  const
{
  return this->id_;
}
/*}}}*/

/*{{{ StartEnd::get_speed(), get_from(), get_to() */
int
StartEnd::get_speed()
  const
{
  return this->speed_;
}

int
StartEnd::get_from()
  const
{
  return this->from_;
}

int
StartEnd::get_to()
  const
{
  return this->to_;
}
/*}}}*/

/*{{{ Car::get_plan_time(), get_priority(), get_preset() */
int
Car::get_plan_time()
  const
{
  return this->plan_time_;
}

int
Car::get_priority()
  const
{
  return this->priority_;
}

int
Car::get_preset()
  const
{
  return this->preset_;
}
/*}}}*/

int
Road::get_length()
  const
{
  return this->length_;
}


void
RunningCar::init(std::vector<RoadOnline*> &p)
{
  this->path_.assign(p.begin(), p.end());
  this->state_ = WAIT;
  this->pos_   = 0;
  
  this->idx_of_current_road_ = -1;
}

bool
RunningCar::move_to_next_road()
{
  if (FINISH == this->state_) {
    return true;
  }

  auto next_road_idx = this->idx_of_current_road_ + 1;
  // TODO:

  return false;
}

int
RunningCar::get_start_time()
  const
{
  return this->start_time_;
}

int
RunningCar::get_pos()
  const
{
  return this->pos_;
}

State
RunningCar::get_state()
  const
{
  return this->state_;
}

void
RunningCar::set_pos(const int p)
{
  this->pos_ = p;
  return;
}

void
RunningCar::set_state(const State s)
{
  this->state_ = s;
  return;
}

void
RunningCar::set_current_road_idx(const std::size_t idx)
{
  this->idx_of_current_road_ = idx;
  return;
}

void
RunningCar::drive(const int speed)
{
  this->pos_ += speed;
  if (this->pos_ >= this->path_[this->idx_of_current_road_].get_length()) {
    this->state_ = WAIT;
  } else {
    this->state_ = FINAL;
  }
  return;
}

/*
 * void
 * RoadInitCarList::put_car_in_init_list(const RunningCar *p_car,
 *                                       const int dir)
 * {
 *   if (0 == dir) {
 *     this->dir_cars_.push_back(p_car);
 *   } else {
 *     this->inv_cars_.push_back(p_car);
 *   }
 *   return;
 * }
 */

/*
 * void
 * RoadInitCarList::remove_car_in_init_list(const std::list<RunningCar>::iterator &it,
 *                                          const int dir)
 * {
 *   if (0 == dir) {
 *     this->dir_cars_.erase(it);
 *   } else {
 *     this->inv_cars_.erase(it);
 *   }
 *   return;
 * }
 */

/*{{{ @deprecated: < RunningCar */
/*
 * bool operator < (const RunningCar &c1,
 *                  const RunningCar &c2)
 * {
 *   return c1.priority_  < c2.priority_ ||
 *         (c1.priority_ == c2.priority_ && c1.start_time_  > c2.start_time_) ||
 *         (c1.priority_ == c2.priority_ && c1.start_time_ == c2.start_time_  && c1.id_ > c2.id_);
 * }
 */
/*}}}*/

/*{{{ @deprecated: < list<RunningCar>::iterator */
bool operator < (const std::list<RunningCar>::iterator &it1,
                 const std::list<RunningCar>::iterator &it2)
{
  return it1->get_priority() < it2->get_priority() ||
        (it1->get_priority() == it2->get_priority() && it1->get_start_time() > it2->get_start_time()) ||
        (it1->get_priority() == it2->get_priority() && it1->get_start_time() == it2->get_start_time() && it1->get_id() > it2->get_id());
}
/*}}}*/

bool
compare_priority(const RunningCar *c1,
                 const RunningCar *c2)
{
  return c1->get_priority()  > c2->get_priority() ||
        (c1->get_priority() == c2->get_priority() && c1->get_start_time()  < c2->get_start_time()) ||
        (c1->get_priority() == c2->get_priority() && c1->get_start_time() == c2->get_start_time() && c1->get_id() < c2->get_id());
}

void
RoadInitCarList::create_sequence()
{
  dir_cars_.sort(compare_priority);
  if (1 == this->is_duplex_) {
    inv_cars_.sort(compare_priority);
  }
  return;
}

/*
 * bool
 * move_to_next_road(RunningCar &car)
 * {
 *   if (car.state_ == FINISH) {
 *     return true;
 *   }
 *   // TODO:
 *   return true;
 * }
 */

void
RoadOnline::drive_just_current_road(std::size_t channel,
                                    std::vector<std::vector<RunningCar*>> &cars)
{
  if (channel >= cars.size()) {
    return;
  }

  // XXX: prev_pos
  int v = 0, prev_pos = 0x3f3f3f3f;
  State prev_state = WAIT;
  for (auto c : cars[channel]) {
    v = std::min(c->get_speed(), this->get_speed());
    if (v > prev_pos - c->get_pos()) {
      v = prev_pos - c->get_pos();
      c->set_state(prev_state);
    } else {
      c->set_state(FINAL);
    }
    c->drive(v);
    prev_pos   = c->get_pos();
    prev_state = c->get_state();
  }

  return;
}

void
RoadOnline::drive_just_current_road(std::size_t channel,
                                    const int start_cross)
{
  if (start_cross == this->from_) {
    drive_just_current_road(channel, this->dir_on_running_cars_);
  }

  if (1 == this->is_duplex_ && start_cross == this->to_) {
    drive_just_current_road(channel, this->inv_on_running_cars_);
  }
  
  return;
}

// @deprecated
/*
 * bool
 * RoadOnline::run_to_road(RunningCar* c)
 * {
 *   if (nullptr == c) return false;
 * 
 *   int v = std::min(c->get_speed(), this->get_speed());
 *   for (auto &channel : dir_on_running_cars_) {
 *     if (channel.back()->get_pos() <= 1){
 *       continue;
 *     }
 * 
 *     if (channel.back()->get_pos() <= v && channel.back()->get_state() == WAIT) {
 *       continue;
 *     }
 * 
 *     v = std::min(v, channel.back()->get_pos() - 1);
 * 
 *     c->set_pos(v);
 *     c->set_state(channel.back()->get_state());
 *     c->set_current_road_idx(0);
 *     channel.push_back(c);
 *     return true;
 *   }
 * 
 *   return false;
 * }
 */

bool
RoadOnline::run_to_road(RunningCar* c,
                       std::vector<std::vector<RunningCar*>> &running_cars)
{
  if (nullptr == c) return false;

  int v = std::min(c->get_speed(), this->get_speed());
  for (auto &channel : running_cars) {
    if (channel.back()->get_pos() <= 1) {
      continue;
    }

    if (channel.back()->get_pos() <= v && channel.back()->get_state() == WAIT) {
      continue;
    }

    v = std::min(v, channel.back()->get_pos() - 1);

    c->set_pos(v);
    c->set_state(channel.back()->get_state());
    c->set_current_road_idx(0);
    channel.push_back(c);
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
