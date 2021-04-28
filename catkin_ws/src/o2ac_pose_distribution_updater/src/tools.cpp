/*
The function which read stl file and store the data to a fcl::BVHModel<fcl::OBBRSS> class
 */

#include "read_stl.cpp"

void load_BVHModel_from_stl_file(const std::string &file_path, const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &bvhm)
{
  FILE *in=fopen(file_path.c_str(), "r");
  std::vector<std::vector<double>> points;
  std::vector<std::vector<int>> tris;
  read_stl(in, points, tris);
  fclose(in);
  std::vector<fcl::Vec3f> fcl_points(points.size());
  for(int i = 0; i < points.size(); i++){
    fcl_points[i] = fcl::Vec3f(points[i][0], points[i][1], points[i][2]) / 1000.0;
  }
  std::vector<fcl::Triangle> fcl_triangles(tris.size());
  for(int j = 0; j < tris.size(); j++){
    fcl_triangles[j] = fcl::Triangle(tris[j][0], tris[j][1], tris[j][2]);
 }
  bvhm->beginModel(fcl_points.size(), fcl_triangles.size());
  bvhm->addSubModel(fcl_points, fcl_triangles);
  bvhm->endModel();
}
