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
#include <unordered_map>
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

  int   get_start_time() const;
  int   get_pos()        const;
  State get_state()      const;

  void drive(const int speed);

  void set_pos(const int p);
  void set_state(const State s);
  void set_current_road_idx(const std::size_t idx);

protected:
  State               state_;
  std::size_t         idx_of_current_road_;
  int                 pos_;
  int                 start_time_;
  std::vector<Road>   path_;

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
  std::list<RunningCar*> dir_cars_;
  std::list<RunningCar*> inv_cars_;
};

class RoadOnline : virtual public RoadInitCarList {
public:
  // bool run_to_road(RunningCar* c);
  bool run_to_road(RunningCar* c, std::vector<std::vector<RunningCar*>> &running_cars);
  void run_car_in_init_list(int current_time);
  void drive_just_current_road(std::size_t channel, const int dir);

protected:
  std::vector<std::vector<RunningCar*>> dir_on_running_cars_;
  std::vector<std::vector<RunningCar*>> inv_on_running_cars_;

private:
  void run_car_in_init_list(const int current_time, std::list<RunningCar*> &init_list, std::vector<std::vector<RunningCar*>> &running_cars);
  void drive_just_current_road(std::size_t channel, std::vector<std::vector<RunningCar*>> &cars);
};

/*{{{ class Cross: id, road_id, road_id, road_id, road_id */
class Cross : virtual public Identity {
public:
  Cross(const int i, const int r1, const int r2, const int r3, const int r4)
    : Identity(i)
    , roads_id_({ r1, r2, r3, r4 }) {}

  void init(std::unordered_map<int, RoadOnline*> road_id_to_roadonline);

protected:
  std::vector<int> roads_id_;
  std::vector<RoadOnline*> roads_online_;

private:
  Cross() = default;
};
/*}}}*/

#endif // ifndef _TRAFFIC_HPP_
