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
