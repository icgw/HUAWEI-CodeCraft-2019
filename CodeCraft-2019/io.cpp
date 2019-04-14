/*
 * parser.cpp
 * Copyright (C) 2019 Guowei Chen <icgw@outlook.com>
 *
 * Distributed under terms of the GPL license.
 */

#include <fstream>   // std::ifstream
#include <sstream>   // std::ostringstream
#include <limits>    // std::numeric_limits
#include <string>
#include <iterator>  // std::ostream_iterator
#include <algorithm> // std::copy
#include <vector>

/*{{{ get integer vector from file input stream */
void
get_vec_from_ifs(std::ifstream &fin,           // IN:  string " 1, 2, 3, ...)"
                 const std::size_t sz,         // IN:  the size of the vector
                 std::vector<int> &iv)         // OUT: vector { 1, 2, 3, ... }
{
  char skip;
  for (int i = 0; i < (int) sz; ++i) {
    fin >> iv[i] >> skip;
  }
  return;
}

void get_vec_from_ifs(std::ifstream &fin,
                      std::vector<int> &iv)
{
  std::string s; getline(fin, s);
  std::stringstream ss(s);

  char skip; int num;
  iv.clear();
  while (ss >> num) {
    iv.push_back(num); ss >> skip;
  }
  return;
}
/*}}}*/

// read file from path and convert each line to vector.
void
read_from_file(const std::string &file_path,       // IN:  the path name of file
               const std::size_t column,           // IN:  the number of variable for each line
               std::vector<std::vector<int>> &iv)  // OUT: the vector of integer vector
{
  std::ifstream fin;
  fin.open(file_path, std::fstream::in);
  if (!fin) return;

  char st;
  while (fin >> st) {
    if ('#' == st) {
      fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    } else {
      std::vector<int> tmp_vec(column);
      get_vec_from_ifs(fin, column, tmp_vec);
      iv.push_back(tmp_vec);
    }
  }
  fin.close();
}

void
read_from_file(const std::string &file_path,
               std::vector<std::vector<int>> &iv)
{
  std::ifstream fin;
  fin.open(file_path, std::fstream::in);
  if (!fin) return;

  std::vector<int> tmp_vec;
  char st;
  while (fin >> st) {
    if ('#' == st) {
      fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    } else {
      get_vec_from_ifs(fin, tmp_vec);
      iv.push_back(tmp_vec);
    }
  }
  fin.close();
  return;
}

/*{{{ convert integer vector into string.*/
void
get_str_from_ivec(const std::vector<int> &iv, // IN:  vector { 1, 2, 3, ... }
                  std::string &s)             // OUT: string "(1, 2, 3, ...)"
{
  std::ostringstream oss;
  oss << "(";
  if (!iv.empty()) {
    std::copy(iv.begin(), iv.end() - 1, std::ostream_iterator<int>(oss, ", "));
    oss << iv.back();
  }
  oss << ")";
  s = oss.str();
  return;
}
/*}}}*/

// write the vector of integer vector into a file.
void
write_to_file(const std::string &file_path,              // IN: the output path of file.
              const std::vector<std::vector<int>> &data) // IN: the output vector.
{
  std::ofstream fout;
  fout.open(file_path, std::fstream::out);
  if (fout.is_open()) {
    std::string s;
    for (auto &v : data) {
      get_str_from_ivec(v, s);
      fout << s << "\n";
    }
    fout.close();
  }
  return;
}
