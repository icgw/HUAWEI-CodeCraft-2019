/*
 * parser.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

// #include <iostream>

#include <vector>
#include <unordered_map>
#include <tuple>

void
process_crosses(std::vector<std::vector<int>> &crosses,
                std::unordered_map<int, int> &cross_id_to_index)
{
  auto sz = crosses.size();
  for (auto idx = 0; idx < sz; ++idx) {
    cross_id_to_index[crosses[idx][0]] = idx;
  }
  return;
}

void
process_roads(std::unordered_map<int, int> &cross_id_to_index, // IN: the map of cross_id to its index.
              std::size_t num_of_nodes,                        // IN: the size of crosses (or node).
              std::vector<std::vector<int>> &roads,            // IN: the original format road data.
              std::vector<std::vector<std::tuple<int, int, int, int>>> &from_to_road_info)
                                                               // OUT: [from][to](id, length, speed, channel)
{
  from_to_road_info.resize(num_of_nodes, 
    std::vector<std::tuple<int, int, int, int>>(num_of_nodes, { -1, 0, 0, 0 }));
  for (auto &rd : roads) {
    auto from = cross_id_to_index[rd[4]];
    auto to   = cross_id_to_index[rd[5]];
    from_to_road_info[from][to] = std::make_tuple(rd[0], rd[1], rd[2], rd[3]);
    if (rd[6] == 1) {
      from_to_road_info[to][from] = std::make_tuple(rd[0], rd[1], rd[2], rd[3]);
    }
  }
  return;
}

/*
 * int main()
 * {
 *   std::vector<std::vector<int>> crs {
 *     { 1, 501, 513, -1, -1 },
 *     { 2, 502, -1, 502, 514}
 *   };
 * 
 *   std::vector<std::vector<int>> rds {
 *     {501, 10, 6, 5, 1, 2, 1},
 *   };
 * 
 *   std::unordered_map<int, int> m;
 *   process_crosses(crs, m);
 * 
 *   std::vector<std::vector<std::tuple<int, int, int, int>>> ftri;
 *   process_roads(m, 2, rds, ftri);
 * 
 *   auto from = 0, to = 1;
 *   auto t1 = ftri[0][1];
 *   std::cout << std::get<0>(t1) << std::endl;
 *   std::cout << std::get<1>(t1) << std::endl;
 *   std::cout << std::get<2>(t1) << std::endl;
 *   std::cout << std::get<3>(t1) << std::endl;
 * 
 *   auto t2 = ftri[1][0];
 *   std::cout << std::get<0>(t2) << std::endl;
 *   std::cout << std::get<1>(t2) << std::endl;
 *   std::cout << std::get<2>(t2) << std::endl;
 *   std::cout << std::get<3>(t2) << std::endl;
 * 
 *   return 0;
 * }
 */
