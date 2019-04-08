/*
 * judge.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include "judge.hpp"

void
Judge::drive_just_current_road()
{
  for (auto &rd : this->roads_) {
    rd.drive_just_current_road();
  }
  return;
}

void
Judge::drive_car_init_list(const int current_time,
                           const bool is_priority)
{
  for (auto &rd : this->roads_) {
    rd.run_car_in_init_list(current_time, is_priority);
  }
  return;
}

void
Judge::create_car_sequence()
{
  for (auto &rd : this->roads_) {
    rd.create_car_in_wait_sequence();
  }
  return;
}

bool
Judge::drive_car_in_wait_state(const int current_time)
{
  // XXX: prev_num_of_wait_car may be exist better implementation.
  int prev_num_of_wait_car = 0x7fffffff;
  while (true) {
    int num_of_wait_car = 0;
    for (auto &c : this->crosses_) {
      for (auto r : c.get_roads()) {
        RunningCar* car;
        while ((car = r->get_front_car_from_wait_sequence(c.get_id())) != nullptr) {
          if (car->is_conflict()) {
            // XXX: something error.
            break;
          }

          auto prev_channel = car->get_current_road_channel();
          if (car->move_to_next_road()) {
            r->drive_just_current_road(prev_channel, c.get_id(), true);
            r->create_car_in_wait_sequence();
            r->run_car_in_init_list(current_time, true);
            // XXX: addition.
            r->pop_front_car_from_wait_sequence(c.get_id());
          } else {
            break;
          }
        }
        num_of_wait_car += r->get_num_of_wait_cars(c.get_id());
      }
    }
    if (num_of_wait_car >= prev_num_of_wait_car) {
      break;
    }

    prev_num_of_wait_car = num_of_wait_car;
  }

  if (prev_num_of_wait_car > 0) {
    // XXX: logging which cross deadlock??
    return false; // deadlock;
  }

  return true;
}

bool
Judge::is_finish()
{
  for (auto &c : this->cars_) {
    if (FINISH != c.get_state()) {
      return false;
    }
  }
  return true;
}

void
Judge::init_car_road_cross(const std::string car_path,
                           const std::string road_path,
                           const std::string cross_path)
{
  std::vector<std::vector<int>> cars, roads, crosses;
  read_from_file(car_path   , CAR_COLUMN   , cars);
  read_from_file(road_path  , ROAD_COLUMN  , roads);
  read_from_file(cross_path , CROSS_COLUMN , crosses);

  for (auto &c : cars) {
    this->cars_.push_back(RunningCar(c[0], c[1], c[2], c[3], c[4], c[5], c[6]));
  }
  std::sort(this->cars_.begin(), this->cars_.end(),
      [](const RunningCar &a, const RunningCar &b) -> bool {
        return a.get_id() < b.get_id();
      });

  for (auto &r : roads) {
    this->roads_.push_back(RoadOnline(r[0], r[1], r[2], r[3], r[4], r[5], r[6]));
  } 
  std::sort(this->roads_.begin(), this->roads_.end(),
      [](const RoadOnline &a, const RoadOnline &b) -> bool {
        return a.get_id() < b.get_id();
      });

  for (auto &cs : crosses) {
    this->crosses_.push_back(Cross(cs[0], cs[1], cs[2], cs[3], cs[4]));
  }
  std::sort(this->crosses_.begin(), this->crosses_.end(),
      [](const Cross &a, const Cross &b) -> bool {
        return a.get_id() < b.get_id();
      });

  return;
}

