/*
 * model.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */


#include <limits>
#include "model.hpp"

Feedback
Model::dijkstra(Info2Point &in2p)
{
  std::vector<double> dist(this->graph_.get_size(),
                           std::numeric_limits<double>::infinity());
  auto st = in2p.start_time;

  return {};
}
