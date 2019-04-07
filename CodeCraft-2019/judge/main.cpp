/*
 * main.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <iostream>
#include "judge.hpp"

int main()
{
  Judge scheduler("", "", "", "");

  int timer = 0;
  while (true) {
    ++timer;
    scheduler.drive_just_current_road();
    scheduler.drive_car_init_list(timer, true);
    scheduler.create_car_sequence();
    if (!scheduler.drive_car_in_wait_state(timer)) {
      // XXX: deadlock
      return -1;
    }

    scheduler.drive_car_init_list(timer, false);

    if (scheduler.is_finish()) {
      // XXX: all cars finished.
      return 0;
    }
  }

  return 0;
}
