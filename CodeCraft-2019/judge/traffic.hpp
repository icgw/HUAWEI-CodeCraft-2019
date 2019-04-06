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
#include <utility> // std::pair
#include <unordered_map>
#include "common.hpp"

/*{{{ class Car: id, from, to, speed, plan_time, priority, preset */
class Car : virtual public StartEnd {
public:
  Car(int i, int from, int to, int speed, int plan_time, int priority, int preset)
    : StartEnd(i, speed, from, to)
    , plan_time_(plan_time)
    , priority_(priority)
    , preset_(preset) {}

  int get_plan_time() const;
  int get_priority()  const;
  int get_preset()    const;

protected:
  int plan_time_;
  int  priority_;
  int    preset_;

private:
  Car() = default;
};
/*}}}*/

/*{{{ class Road: id, length, speed, channel, from, to, is_duplex */
class Road : virtual public StartEnd {
public:
  Road(int i, int len, int speed, int channel, int from, int to, int is_duplex)
    : StartEnd(i, speed, from, to)
    , length_(len)
    , channel_(channel)
    , is_duplex_(is_duplex) {}

  int get_length() const;

protected:
  int     length_;
  int    channel_;
  bool is_duplex_;
};
/*}}}*/

/*{{{ class Cross: id, road_id, road_id, road_id, road_id */
class RoadOnline;
// XXX:
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

class RoadOnline;
class Cross;
class RunningCar : virtual public Car {
public:
  int   get_start_time()       const;
  int   get_current_road_pos() const;
  State get_state()            const;
  int   get_current_road_channel() const;

  void set_current_road_pos(const int p);
  void set_current_road_channel(const int channel);
  void set_state(const State s);
  void set_current_road_idx(const std::size_t idx);

  void init(std::vector<RoadOnline*> &p);
  bool move_to_next_road();

  void drive(const int speed);

protected:
  int           start_time_;
  State              state_;

  int  idx_of_current_road_;

  int     current_road_pos_;
  int        next_road_pos_;

  int current_road_channel_;

  std::vector<RoadOnline*>   path_;
  std::vector<Cross*>        start_cross_id_sequence_;

private:
  RunningCar() = default;
};

class RoadInitCarList : virtual public Road {
public:
  void create_car_sequence();

protected:
  std::list<RunningCar*> dir_cars_;
  std::list<RunningCar*> inv_cars_;
};

class RoadOnline : virtual public RoadInitCarList {
public:
  bool run_to_road(RunningCar* c, std::vector<std::vector<RunningCar*>> &running_cars);
  void run_car_in_init_list(int current_time);
  void drive_just_current_road(const int channel, const int start_cross_id);

  // @deprecated
  // void drive_just_current_road(std::size_t channel, const int dir);

  bool is_final_filled(const int start_cross_id);
  std::pair<int, int> select_valid_channel(const int start_cross_id);

  // @deprecated
  // bool is_filled(const int start_cross_id);

protected:
  std::vector<std::vector<RunningCar*>> dir_on_running_cars_;
  std::vector<std::vector<RunningCar*>> inv_on_running_cars_;

private:
  void run_car_in_init_list(const int current_time, std::list<RunningCar*> &init_list, std::vector<std::vector<RunningCar*>> &running_cars);
  // @deprecated
  void drive_just_current_road(const int channel, std::vector<std::vector<RunningCar*>> &cars);
};

#endif // ifndef _TRAFFIC_HPP_
