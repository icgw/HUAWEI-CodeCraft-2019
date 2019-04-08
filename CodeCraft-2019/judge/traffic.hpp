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
#include <algorithm>
#include "common.hpp"

/*{{{ class Car: id, from, to, speed, plan_time, priority, preset */
class Car : public StartEnd {
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

inline int
Car::get_plan_time()
  const
{
  return this->plan_time_;
}

inline int
Car::get_priority()
  const
{
  return this->priority_;
}

inline int
Car::get_preset()
  const
{
  return this->preset_;
}
/*}}}*/

/*{{{ class Road: id, length, speed, channel, from, to, is_duplex */
class Road : public StartEnd {
public:
  Road(int i, int len, int speed, int channel, int from, int to, int is_duplex)
    : StartEnd(i, speed, from, to)
    , length_(len)
    , channel_(channel)
    , is_duplex_(is_duplex) {}

  int get_length() const;
  int get_duplex() const;

protected:
  int     length_;
  int    channel_;
  bool is_duplex_;
};

inline int
Road::get_length()
  const
{
  return this->length_;
}

inline int
Road::get_duplex()
  const
{
  return this->is_duplex_;
}
/*}}}*/

/*{{{ class Cross: id, road_id, road_id, road_id, road_id */
class RoadOnline;
// XXX:
class Cross : public Identity {
public:
  Cross(const int i, const int r1, const int r2, const int r3, const int r4)
    : Identity(i)
    , roads_id_({ r1, r2, r3, r4 }) {
      std::sort(this->roads_id_.begin(), this->roads_id_.end());
    }

  std::vector<RoadOnline*> get_roads();
  void init(std::unordered_map<int, RoadOnline*> road_id_to_roadonline);

protected:
  std::vector<int> roads_id_;
  std::vector<RoadOnline*> roads_online_;

private:
  Cross() = default;
};

inline std::vector<RoadOnline*>
Cross::get_roads()
{
  return this->roads_online_;
}
/*}}}*/

/*{{{ class RunningCar*/
class RoadInitCarList;
class RoadOnline;
class Cross;
class RunningCar : virtual public Car {
public:
  RunningCar(int i, int from, int to, int speed, int plan_time, int priority, int preset)
    : Car(i, from, to, speed, plan_time, priority, preset) {}

  int   get_start_time()           const;
  int   get_current_road_pos()     const;
  State get_state()                const;
  int   get_current_road_channel() const;

  bool is_conflict() const;

  void set_current_road_pos(const int p);
  void set_current_road_channel(const int channel);
  void set_state(const State s);
  void set_current_road_idx(const std::size_t idx);

  void init(std::vector<RoadOnline*> &p);
  bool move_to_next_road();

  void drive(const int speed);

protected:
  int                      start_time_;
  int                      idx_of_current_road_;
  int                      current_road_pos_;
  int                      next_road_pos_;
  int                      current_road_channel_;
  State                    state_;

  std::vector<RoadOnline*> path_;
  std::vector<Cross*>      start_cross_id_sequence_;

private:
  RunningCar() = default;
};

inline int
RunningCar::get_start_time()
  const
{
  return this->start_time_;
}

inline int
RunningCar::get_current_road_pos()
  const
{
  return this->current_road_pos_;
}

inline int
RunningCar::get_current_road_channel()
  const
{
  return this->current_road_channel_;
}

inline State
RunningCar::get_state()
  const
{
  return this->state_;
}

inline void
RunningCar::set_current_road_pos(const int p)
{
  this->current_road_pos_ = p;
  return;
}

inline void
RunningCar::set_current_road_channel(const int channel)
{
  this->current_road_channel_ = channel;
  return;
}

inline void
RunningCar::set_state(const State s)
{
  this->state_ = s;
  return;
}

inline void
RunningCar::set_current_road_idx(const std::size_t idx)
{
  this->idx_of_current_road_ = idx;
  return;
}

/*}}}*/

class RoadInitCarList : public Road {
public:
  RoadInitCarList(int id, int len, int speed, int channel, int from, int to, int is_duplex)
    : Road(id, len, speed, channel, from, to, is_duplex) {}
  void create_car_sequence_for_ready_car();

protected:
  std::list<RunningCar*> dir_cars_;
  std::list<RunningCar*> inv_cars_;

private:
  RoadInitCarList() = default;
};

class RoadOnline : virtual public RoadInitCarList {
public:
  RoadOnline(int id, int len, int speed, int channel, int from, int to, int is_duplex)
    : RoadInitCarList(id, len, speed, channel, from, to, is_duplex) {}

  int get_num_of_wait_cars(const int start_cross_id) const;

  bool run_to_road(RunningCar* c, std::vector<std::list<RunningCar*>> &running_cars);
  void run_car_in_init_list(const int current_time, const bool is_priority);

  void drive_just_current_road(const int channel, const int start_cross_id, const bool for_wait_car);
  void drive_just_current_road();

  bool is_final_filled(const int start_cross_id);
  std::pair<int, int> select_valid_channel(const int start_cross_id);

  void remove_car(const int channel, const int start_cross_id, RunningCar* const car);
  void push_back_car(const int channel, const int start_cross_id, RunningCar* const car);

  void create_car_in_wait_sequence();
  void pop_front_car_from_wait_sequence(const int start_cross_id);

  RunningCar* get_front_car_from_wait_sequence(const int start_cross_id);

protected:
  std::vector<std::list<RunningCar*>> dir_on_running_cars_ls_;
  std::list<RunningCar*>              dir_on_waiting_cars_ls_;

  std::vector<std::list<RunningCar*>> inv_on_running_cars_ls_;
  std::list<RunningCar*>              inv_on_waiting_cars_ls_;

private:
  void run_car_in_init_list(const int current_time, const bool is_priority, std::list<RunningCar*> &init_list, std::vector<std::list<RunningCar*>> &running_cars);
  void drive_just_current_road(const int channel, std::vector<std::list<RunningCar*>> &cars, const bool for_wait_car);
};

inline int
RoadOnline::get_num_of_wait_cars(const int start_cross_id)
  const
{
  if (start_cross_id == this->from_) {
    return this->dir_on_waiting_cars_ls_.size();
  }
  else if (start_cross_id == this->to_) {
    return this->inv_on_waiting_cars_ls_.size();
  }
  return 0;
}

inline bool
RunningCar::is_conflict()
  const
{
  // XXX: probably have some bugs.
  auto idx = this->idx_of_current_road_;
  auto cross_id = this->start_cross_id_sequence_[idx]->get_id();
  return this->path_[idx]->get_from() == cross_id ||
        (this->path_[idx]->get_duplex() != 0 && this->path_[idx]->get_to() == cross_id);
}

#endif // ifndef _TRAFFIC_HPP_
