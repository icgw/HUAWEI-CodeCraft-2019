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
RunningCar::init(std::vector<Road> &p)
{
  this->path_.assign(p.begin(), p.end());
  this->state_ = WAIT;
  this->pos_   = 0;
}

void
RunningCar::go_next_road()
{
  ++(this->pos_);
  if (this->pos_ >= path_.size()) {
    this->state_ = FINISH;
  }
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
RunningCar::set_state(const State s)
{
  this->state_ = s;
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

void
RoadInitCarList::put_car_in_init_list(const RunningCar &c,
                                      const int dir)
{
  if (0 == dir) {
    this->dir_cars_.push_back(c);
  } else {
    this->inv_cars_.push_back(c);
  }
  return;
}

void
RoadInitCarList::remove_car_in_init_list(const std::list<RunningCar>::iterator &it,
                                         const int dir)
{
  if (0 == dir) {
    this->dir_cars_.erase(it);
  } else {
    this->inv_cars_.erase(it);
  }
  return;
}

/*{{{ @deprecated: < RunningCar */
bool operator < (const RunningCar &c1,
                 const RunningCar &c2)
{
  return c1.priority_  < c2.priority_ ||
        (c1.priority_ == c2.priority_ && c1.start_time_  > c2.start_time_) ||
        (c1.priority_ == c2.priority_ && c1.start_time_ == c2.start_time_  && c1.id_ > c2.id_);
}
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
compare_priority(const RunningCar &c1,
                 const RunningCar &c2)
{
  return c1.get_priority()  > c2.get_priority() ||
        (c1.get_priority() == c2.get_priority() && c1.get_start_time()  < c2.get_start_time()) ||
        (c1.get_priority() == c2.get_priority() && c1.get_start_time() == c2.get_start_time() && c1.get_id() < c2.get_id());
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

bool
move_to_next_road(RunningCar &car)
{
  if (car.state_ == FINISH) {
    return true;
  }
  // TODO:
  return true;
}

void
OnlineRoad::run_car_in_init_list(int current_time)
{
  return;
}

void
OnlineRoad::drive_just_current_road(std::size_t channel,
                                    std::vector<std::vector<RunningCar>> &cars)
{
  if (channel >= cars.size()) {
    return;
  }

  int v = 0, prev_pos = 0x3f3f3f3f;
  State prev_state = WAIT;
  for (auto &c : cars[channel]) {
    v = std::min(c.get_speed(), this->get_speed());
    if (v > prev_pos - c.get_pos()) {
      v = prev_pos - c.get_pos();
      c.set_state(prev_state);
    } else {
      c.set_state(FINAL);
    }
    c.drive(v);
    prev_pos   = c.get_pos();
    prev_state = c.get_state();
  }

  return;
}

void
OnlineRoad::drive_just_current_road(std::size_t channel,
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
