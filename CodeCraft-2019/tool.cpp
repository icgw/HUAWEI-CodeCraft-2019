/*
 * tool.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <iostream>
#include <vector>

#include <cmath>

void
make_increase(std::vector<int> &sequence)
{
  if (sequence.size() <= 0) return;
  int tmax = sequence[0];
  for (auto &v : sequence) {
    v    = std::max(tmax, v);
    tmax = std::max(tmax, v);
  }
  return;
}

// magical..
void
make_logistics_like(std::vector<int> &nums, double L)
{
  // nums.size() >= 3.
  int m = (nums.size() + 1) / 2;
  double k = pow(L / nums[0] + 1.0, 1.0 / m);
  double e = nums[0];

  for (auto i = 1; i < nums.size(); ++i) {
    e = L / ((L / e - 1) / k + 1);
    nums[i] = std::max(nums[i], (int) e);
  }
  return ;
}

/*
 * int main()
 * {
 *   std::vector<int> a { 1, 3, 1, 2, 4, 5, 1, 10, 9, 16 };
 *   for (auto x : a) {
 *     std::cout << x << ", ";
 *   }
 *   std::cout << "\n";
 * 
 *   // make_increase(a);
 *   make_logistics_like(a, 500.0);
 *   for (auto x : a) {
 *     std::cout << x << ", ";
 *   }
 *   std::cout << "\n";
 * 
 *   return 0;
 * }
 */
