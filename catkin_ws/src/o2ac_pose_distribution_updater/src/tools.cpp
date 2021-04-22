#include <pcl/io/vtk_lib_io.h>

void load_BVHModel_from_stl_file(const std::string &file_path, const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &bvhm)
{
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  pcl::io::loadPolygonFileSTL(file_path, *mesh);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(mesh->cloud, *pcd);
  std::vector<fcl::Vec3f> points(pcd->size());
  for(int i = 0; i < pcd->size(); i++){
    points[i] = fcl::Vec3f((*pcd)[i].x, (*pcd)[i].y, (*pcd)[i].z) / 1000.0;
  }
  std::vector<fcl::Triangle> triangles(mesh->polygons.size());
  for(int j = 0; j < mesh->polygons.size(); j++){
    auto vertices = mesh->polygons[j].vertices;
    assert(vertices.size() == 3);
    assert(0<= vertices[0] && vertices[0] < points.size());
    assert(0<= vertices[1] && vertices[1] < points.size());
    assert(0<= vertices[2] && vertices[2] < points.size());
    triangles[j] = fcl::Triangle(vertices[0], vertices[1], vertices[2]);
 }
  bvhm->beginModel(points.size(), triangles.size());
  bvhm->addSubModel(points, triangles);
  bvhm->endModel();
}

