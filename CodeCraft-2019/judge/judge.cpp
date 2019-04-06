/*
 * judge.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include "judge.hpp"

void
Judge::drive_just_current_road()
{
  for (auto &rd : this->roads_) {
    rd.drive_just_current_road();
  }
  return;
}

bool
Judge::is_finish()
{
  for (auto &c : this->cars_) {
    if (FINISH != c.get_state()) {
      return false;
    }
  }
  return true;
}
