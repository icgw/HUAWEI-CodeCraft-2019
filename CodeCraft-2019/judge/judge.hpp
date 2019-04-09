/*
 * judge.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _JUDGE_HPP_
#define _JUDGE_HPP_

#include <algorithm>
#include <vector>
#include <string>

#include <unordered_map>
#include <utility>
#include <map>

#include "traffic.hpp"
#include "../io.hpp"

class Judge {
public:
  // TODO: process input data.
  Judge(std::string car_path, std::string road_path, std::string cross_path, std::string preset_path, std::string answer_path);

  void drive_just_current_road();
  void drive_car_init_list(const int current_time, const bool is_priority);
  void create_car_sequence();
  bool drive_car_in_wait_state(const int current_time);
  bool is_finish();

  int get_all_schedule_time();

  void deadlock_info();

private:
  Judge() = default;

  void init_car_road_cross(const std::string car_path, const std::string road_path, const std::string cross_path);
  void init_preset_and_answer_path(const std::string preset_path, const std::string answer_path);

  void init_cars_path(std::vector<std::vector<int>> &schedule, const int b_preset);

  // sort cross by id ascending. and each road id ascending.
  std::vector<Cross>      crosses_;
  std::vector<RoadOnline> roads_;
  std::vector<RunningCar> cars_;

  // map id -> pointer
  std::unordered_map<int, RunningCar*> m_id_to_pcar_;
  std::unordered_map<int, RoadOnline*> m_id_to_proad_;
  std::unordered_map<int, Cross*>      m_id_to_pcross_;

  // map (road1, road2) --> cross
  std::map<std::pair<RoadOnline*, RoadOnline*>, Cross*> m_pair_proads_to_pcross_;

  // Deadlock info.
  std::vector<int> deadlock_cross_id_;
  std::vector<int> waiting_cars_id_;
  std::vector<int> overload_road_id_;
};

inline
Judge::Judge(std::string car_path,
             std::string road_path,
             std::string cross_path,
             std::string preset_path,
             std::string answer_path)
{
  this->init_car_road_cross(car_path, road_path, cross_path);
  this->init_preset_and_answer_path(preset_path, answer_path);
}

#endif // ifndef _JUDGE_HPP_
