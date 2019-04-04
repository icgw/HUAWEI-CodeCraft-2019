/*
 * judge.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _JUDGE_HPP_
#define _JUDGE_HPP_

namespace judge {
  void drive_just_current_road();
  void drive_car_init_list(bool b);
  void create_car_sequence();
  bool drive_car_in_wait_state();
  bool is_finish();
}

#endif // ifndef _JUDGE_HPP_
