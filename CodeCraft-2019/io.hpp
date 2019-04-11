/*
 * io.hpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#ifndef _IO_HPP_
#define _IO_HPP_

#include <string>
#include <vector>

void read_from_file(const std::string&, const std::size_t, std::vector<std::vector<int>>&);
void read_from_file(const std::string&, std::vector<std::vector<int>>&);

void write_to_file(const std::string&, const std::vector<std::vector<int>>&);

#endif // ifndef _IO_HPP_
