/*
 * main.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <iostream>
#include "common.hpp"

int main()
{
  State a = READY;
  if (a == READY) {
    std::cout << a << std::endl;
  }
  return 0;
}
