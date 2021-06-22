#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include <sys/stat.h>

void read_stl(FILE *in, std::vector<Eigen::Vector3d> &points,
              std::vector<boost::array<int, 3>> &triangles) {
  char comment[80];
  fread(comment, 1, 80, in);
  unsigned int number;
  fread(&number, 4, 1, in);
  fseek(in, 0L, SEEK_END);
  unsigned int size = ftell(in);
  fseek(in, 0, SEEK_SET);
  if (size != number * 50 + 84) {
    // ASCII mode
    char c;
    while ((c = fgetc(in)) != '\n')
      ;
    using tuple = std::tuple<std::string, std::string, std::string>;
    std::map<tuple, int> index;
    while (1) {
      char next_word[10];
      fscanf(in, "%s", next_word);
      if (strcmp(next_word, "endsolid") == 0)
        break;
      while ((c = fgetc(in)) != '\n')
        ;
      while ((c = fgetc(in)) != '\n')
        ;
      boost::array<int, 3> triangle;
      for (int i = 0; i < 3; i++) {
        char string_x[99], string_y[99], string_z[99];
        fscanf(in, "%s%s%s%s", next_word, string_x, string_y, string_z);
        tuple t = std::make_tuple(std::string(string_x), std::string(string_y),
                                  std::string(string_z));
        if (index.find(t) == index.end()) {
          index[t] = points.size();
          Eigen::Vector3d point;
          point << atof(string_x), atof(string_y), atof(string_z);
          points.push_back(point);
        }
        triangle[i] = index[t];
      }
      triangles.push_back(triangle);
      while ((c = fgetc(in)) != '\n')
        ;
      while ((c = fgetc(in)) != '\n')
        ;
      while ((c = fgetc(in)) != '\n')
        ;
    }
  } else {
    // Binary mode
    char comment[80];
    fread(comment, 1, 80, in);
    unsigned int number;
    fread(&number, 4, 1, in);
    triangles = std::vector<boost::array<int, 3>>(number);
    using tuple = std::tuple<float, float, float>;
    std::map<tuple, int> index;
    for (int triangle_id = 0; triangle_id < number; triangle_id++) {
      float normal[3], coordinates[3][3];
      fread(normal, 4, 3, in);
      for (int i = 0; i < 3; i++) {
        fread(coordinates[i], 4, 3, in);
        tuple t = std::make_tuple(coordinates[i][0], coordinates[i][1],
                                  coordinates[i][2]);
        if (index.find(t) == index.end()) {
          index[t] = points.size();
          Eigen::Vector3d point;
          point << coordinates[i][0], coordinates[i][1], coordinates[i][2];
          points.push_back(point);
        }
        triangles[triangle_id][i] = index[t];
      }
      unsigned short int unused;
      fread(&unused, 2, 1, in);
    }
  }
}

void read_stl_from_file_path(const std::string &file_path,
                             std::vector<Eigen::Vector3d> &points,
                             std::vector<boost::array<int, 3>> &triangles) {
  FILE *in = fopen(file_path.c_str(), "r");
  read_stl(in, points, triangles);
  fclose(in);
}
