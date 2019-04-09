/*
 * judge.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <iostream> // DEBUG
#include <set>      // FOR DEADLOCK INFO
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
  int num_of_wait_car;
  while (true) {
    num_of_wait_car = 0;
    for (auto &c : this->crosses_) {
      for (auto r : c.get_roads()) {
        RunningCar* car;
        while ((car = r->get_front_car_from_wait_sequence(c.get_id())) != nullptr) {
          if (car->is_conflict()) {
            // XXX: something error.
            std::cout << "conflict.\n";
            break;
          }

          auto prev_channel = car->get_current_road_channel();
          if (car->move_to_next_road()) {
            if (car->get_state() == WAIT) {
              std::cout << "STILL WAIT.\n";
            }
            r->drive_just_current_road(prev_channel, c.get_id(), true);
            r->create_car_in_wait_sequence();
            r->run_car_in_init_list(current_time, true);

            // XXX: something wrong?
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

    if (num_of_wait_car == 0) {
      return true;
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
  /*
   * int count = 0; bool flag = true;
   * for (auto &c : this->cars_) {
   *   if (FINISH != c.get_state()) {
   *     flag = false;
   *   } else {
   *     count++;
   *   }
   * }
   * std::cout << count << std::endl;
   * return flag;
   */

  for (auto &c : this->cars_) {
    if (FINISH != c.get_state()) {
      return false;
    }
  }
  return true;
}

int
Judge::get_all_schedule_time()
{
  int all_schedule_time = 0;
  for (auto &c : this->cars_) {
    all_schedule_time += c.get_end_time();
  }
  return all_schedule_time;
}

void
Judge::init_car_road_cross(const std::string car_path,
                           const std::string road_path,
                           const std::string cross_path)
{
  // id, from, to, speed, plan_time, priority, preset
  constexpr int CAR_COLUMN   = 7;
  // id, length, speed, channel, from, to, is_duplex
  constexpr int ROAD_COLUMN  = 7;
  // id, road_id, road_id, road_id, road_id
  constexpr int CROSS_COLUMN = 5;

  std::vector<std::vector<int>> cars, roads, crosses;
  read_from_file(car_path   , CAR_COLUMN   , cars);
  read_from_file(road_path  , ROAD_COLUMN  , roads);
  read_from_file(cross_path , CROSS_COLUMN , crosses);

/*{{{ for cars_, m_id_to_pcar_*/
  for (auto &c : cars) {
    this->cars_.push_back(RunningCar(c[0], c[1], c[2], c[3], c[4], c[5], c[6]));
  }
  std::sort(this->cars_.begin(), this->cars_.end(),
      [](const RunningCar &a, const RunningCar &b) -> bool {
        return a.get_id() < b.get_id();
      });
  for (auto &c : this->cars_) {
    this->m_id_to_pcar_[c.get_id()] = &c;
  }
/*}}}*/

/*{{{ for roads_, m_id_to_pcar_*/
  for (auto &r : roads) {
    this->roads_.push_back(RoadOnline(r[0], r[1], r[2], r[3], r[4], r[5], r[6]));
  }
  std::sort(this->roads_.begin(), this->roads_.end(),
      [](const RoadOnline &a, const RoadOnline &b) -> bool {
        return a.get_id() < b.get_id();
      });
  for (auto &r : this->roads_) {
    this->m_id_to_proad_[r.get_id()] = &r;
  }
/*}}}*/

/*{{{ for crsses_, m_id_to_pcross_*/
  for (auto &cs : crosses) {
    this->crosses_.push_back(Cross(cs[0], cs[1], cs[2], cs[3], cs[4]));
  }
  std::sort(this->crosses_.begin(), this->crosses_.end(),
      [](const Cross &a, const Cross &b) -> bool {
        return a.get_id() < b.get_id();
      });
  for (auto &cs : this->crosses_) {
    this->m_id_to_pcross_[cs.get_id()] = &cs;
  }
/*}}}*/

  // initiating crosses and ..
  for (auto &cs : this->crosses_) {
    cs.init(this->m_id_to_proad_);
    // initiating m_pair_roads_to_cross_;
    auto roads = cs.get_roads();
    auto sz    = roads.size();
    for (auto i = 0; i < sz; ++i) {
      for (auto j = 0; j < sz; ++j) {
        if (i == j) continue;
        this->m_pair_proads_to_pcross_[std::make_pair(roads[i], roads[j])] = &cs;
      }
    }
  }

  return;
}

void
Judge::init_cars_path(std::vector<std::vector<int>> &schedule,
                      const int b_preset) // IN: 1, preset; IN: 0, not preset.
{
  for (auto &v : schedule) {
    auto id = v[0];
    auto is_preset = this->m_id_to_pcar_[id]->get_preset();
    if (b_preset != is_preset) {
      // XXX: logging something error.
      continue;
    }

    std::vector<RoadOnline*> this_path;
    this_path.clear();
    auto sz = v.size();
    for (auto i = 2; i < sz; ++i) {
      this_path.push_back(this->m_id_to_proad_[v[i]]);
    }
    this->m_id_to_pcar_[id]->init(v[1], this_path, this->m_pair_proads_to_pcross_, this->m_id_to_pcross_);
  }
  return;
}

void
Judge::init_preset_and_answer_path(const std::string preset_path,
                                   const std::string answer_path)
{
  std::vector<std::vector<int>> preset, answer;
  read_from_file(preset_path, preset);
  read_from_file(answer_path, answer);

  this->init_cars_path(preset, 1);
  this->init_cars_path(answer, 0);

  for (auto &cr : this->cars_) {
    RoadOnline* road = cr.get_src_road();
    if (road) {
      road->push(&cr, cr.get_from());
    }
  }

  return;
}

void
Judge::deadlock_info()
{
  this->deadlock_cross_id_.clear();
  this->waiting_cars_id_.clear();
  this->overload_road_id_.clear();

  int n = 0;
  for (auto &cr : this->cars_) {
    if (WAIT != cr.get_state()) {
      continue;
    }
    auto car_id = cr.get_id();
    auto road_id = cr.get_current_road_id();
    auto goto_cross_id = cr.get_current_road_goto_id();
    this->deadlock_cross_id_.push_back(goto_cross_id);
    this->waiting_cars_id_.push_back(car_id);
    this->overload_road_id_.push_back(road_id);
    ++n;
  }

  /*
   * for (auto i = 0; i < n; ++i) {
   *   std::cout << "#Car: " << this->waiting_cars_id_[i] << ", "
   *             << "#Goto_cross: " << this->deadlock_cross_id_[i] << ", "
   *             << "#Current_road: " << this->overload_road_id_[i] << ".\n";
   * }
   */
  std::cout << "Total Car: " << n << "\n";

  std::set<int> cross_id(this->deadlock_cross_id_.begin(), this->deadlock_cross_id_.end());
  std::cout << "Total cross: " << cross_id.size() << "\n";
  for (auto cs : cross_id) {
    std::cout << cs << "\n";
  }
  std::set<int> road_id(this->overload_road_id_.begin(), this->overload_road_id_.end());
  std::cout << "Total road: " << road_id.size() << "\n";
  for (auto rd : road_id) {
    std::cout << rd << "\n";
  }

  return;
}
