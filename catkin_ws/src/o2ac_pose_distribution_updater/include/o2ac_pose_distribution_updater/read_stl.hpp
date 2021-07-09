#include <Eigen/Core>
#include <boost/array.hpp>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <tuple>
#include <vector>

void read_stl(FILE *in, std::vector<Eigen::Vector3d> &points,
              std::vector<boost::array<int, 3>> &triangles);
void read_stl_from_file_path(const std::string &file_path,
                             std::vector<Eigen::Vector3d> &points,
                             std::vector<boost::array<int, 3>> &triangles);
