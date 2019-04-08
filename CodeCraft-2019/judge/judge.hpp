/*
 * judge.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _JUDGE_HPP_
#define _JUDGE_HPP_

#include <iostream> // for testing

#include <algorithm>
#include <vector>
#include <string>

#include "traffic.hpp"
#include "../io.hpp"

// id, from, to, speed, plan_time, priority, preset
constexpr int CAR_COLUMN   = 7;

// id, length, speed, channel, from, to, is_duplex
constexpr int ROAD_COLUMN  = 7;

// id, road_id, road_id, road_id, road_id
constexpr int CROSS_COLUMN = 5;

class Judge {
public:
  // TODO: process input data.
  Judge(std::string car_path, std::string road_path, std::string cross_path, std::string preset_path, std::string answer_path);

  void drive_just_current_road();
  void drive_car_init_list(const int current_time, const bool is_priority);
  void create_car_sequence();
  bool drive_car_in_wait_state(const int current_time);
  bool is_finish();

private:
  Judge() = default;

  void init_car_road_cross(const std::string car_path, const std::string road_path, const std::string cross_path);

  // sort cross by id ascending. and each road id ascending.
  std::vector<Cross>      crosses_;
  std::vector<RoadOnline> roads_;
  std::vector<RunningCar> cars_;
};

inline
Judge::Judge(std::string car_path,
             std::string road_path,
             std::string cross_path,
             std::string preset_path,
             std::string answer_path)
{
  this->init_car_road_cross(car_path, road_path, cross_path);
  Cross cs      = this->crosses_[0]; std::cout << cs.get_id() << std::endl;
  RoadOnline rd = this->roads_[0];   std::cout << rd.get_id() << std::endl;
  RunningCar cr = this->cars_[0];    std::cout << cr.get_id() << std::endl;
}

#endif // ifndef _JUDGE_HPP_
