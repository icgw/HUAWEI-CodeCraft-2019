/*
 * main.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <iostream>
#include "judge.hpp"

int main(int argc, char *argv[])
{
  if (argc < 6) {
    // XXX: logging error.
    exit(1);
  }

  std::string carPath          (argv[1]);
  std::string roadPath         (argv[2]);
  std::string crossPath        (argv[3]);
  std::string presetAnswerPath (argv[4]);
  std::string answerPath       (argv[5]);

  Judge scheduler(carPath, roadPath, crossPath, presetAnswerPath, answerPath);

  int timer = 0;
  while (true) {
    ++timer;
    scheduler.drive_just_current_road();
    scheduler.drive_car_init_list(timer, true);
    scheduler.create_car_sequence();
    if (!scheduler.drive_car_in_wait_state(timer)) {
      // XXX: deadlock
      std::cout << "deadlock\n" << std::endl;
      return -1;
    }

    scheduler.drive_car_init_list(timer, false);

    if (scheduler.is_finish()) {
      // XXX: all cars finished.
      std::cout << "Original Result: schedule time = " << timer << ", " 
                << "all schedule time = " << scheduler.get_all_schedule_time()
                << "\n";
      return 0;
    }
  }

  return 0;
}
