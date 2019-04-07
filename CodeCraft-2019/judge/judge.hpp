/*
 * judge.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _JUDGE_HPP_
#define _JUDGE_HPP_

#include <vector>
#include <string>
#include "traffic.hpp"

class Judge {
public:
  // TODO: process input data.
  Judge(std::string car_path, std::string cross_path, std::string preset_path, std::string road_path);

  void drive_just_current_road();
  void drive_car_init_list(const int current_time, const bool is_priority);
  void create_car_sequence();
  bool drive_car_in_wait_state(const int current_time);
  bool is_finish();

private:
  Judge() = default;

  // sort cross by id ascending. and each road id ascending.
  std::vector<Cross>      crosses_;
  std::vector<RoadOnline> roads_;
  std::vector<RunningCar> cars_;
};

#endif // ifndef _JUDGE_HPP_
