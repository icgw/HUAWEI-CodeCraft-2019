/*
 * traffic.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _TRAFFIC_HPP_
#define _TRAFFIC_HPP_

#include <vector>
#include <list>
#include <queue>
#include "common.hpp"

/*{{{ class Car: id, from, to, speed, plan_time, priority, preset */
class Car   : virtual public StartEnd {
public:
  int get_plan_time() const;
  int get_priority()  const;
  int get_preset()    const;

protected:
  int  plan_time_;
  int  priority_;
  bool preset_;

private:
  Car() = default;
};
/*}}}*/

/*{{{ class Cross: id, road_id, road_id, road_id, road_id */
class Cross : virtual public Identity {
public:
  Cross(int i) : Identity(i), roads_({ -1, -1, -1, -1}) {}

protected:
  std::vector<int> roads_;

private:
  Cross() = default;
};
/*}}}*/

/*{{{ class Road: id, length, speed, channel, from, to, is_duplex */
class Road  : virtual public StartEnd {
public:
  int get_length() const;

protected:
  int length_;
  int channel_;
  bool is_duplex_;
};
/*}}}*/

class RunningCar : virtual public Car {
  friend bool operator < (const RunningCar &c1, const RunningCar &c2);
  friend bool move_to_next_road(RunningCar &car);

public:
  void init(std::vector<Road> &p);
  void go_next_road();

  int get_start_time() const;
  int get_pos() const;
  State get_state() const;

  void drive(const int speed);

  void set_state(const State s);

protected:
  State               state_;
  int                 start_time_;
  std::vector<Road>   path_;
  std::size_t         idx_of_current_road_;
  int                 pos_;

private:
  RunningCar() = default;
};

class RoadInitCarList : virtual public Road {
  friend bool compare_priority(const RunningCar &c1, const RunningCar &c2);

public:
  void put_car_in_init_list(const RunningCar &c, const int dir);
  void remove_car_in_init_list(const std::list<RunningCar>::iterator &it, const int dir);
  void create_sequence();

protected:
  std::list<RunningCar> dir_cars_;
  std::list<RunningCar> inv_cars_;
};

class OnlineRoad : virtual public RoadInitCarList {
public:
  void run_car_in_init_list(int current_time);
  void drive_just_current_road(std::size_t channel, const int dir);

protected:
  std::vector<std::vector<RunningCar>> dir_on_running_cars_;
  std::vector<std::vector<RunningCar>> inv_on_running_cars_;

private:
  void drive_just_current_road(std::size_t channel, std::vector<std::vector<RunningCar>> &cars);
};

#endif // ifndef _TRAFFIC_HPP_
