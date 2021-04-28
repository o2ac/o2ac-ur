/*
The implementation of the test client for place action

This client read the path of the stl file name representing the gripped object and the number of cases to generate from standard input.
For each case, this client ramdomly generates the gripper pose and the pose of the gripped object.
The generated pose is written in a stl file named "inputX.stl" in the "results" directory where 'X' is the case index.
For each case, the client sends a place action call to the action server such that the object is placed to the ground "z = -0.1".
The pose after placing sent by action server is also written in a stl file named "outputX.stl" in the "results" directory where 'X' is the case index.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include "../conversions.cpp"
#include "../ros_converters.cpp"
#include "o2ac_msgs/updateDistributionAction.h"
#include <random>

namespace{
  const double pi = 3.1416;
  
  std::random_device seed_generator;
  std::default_random_engine engine(seed_generator());
  std::uniform_real_distribution<double> dist(0, 1.0);

  Particle get_random_particle()
  {
    Particle p;
    p<< 0.1 * dist(engine), 0.1 * dist(engine), 0.1 * dist(engine), 2.0 * pi * dist(engine), 2.0 * pi * dist(engine), 2.0 * pi * dist(engine);
    return p;
  }
}


void MeshTransform(const pcl::PolygonMesh::Ptr &mesh,
		   const Eigen::Transform<double, 3, 2> &transform)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh->cloud, cloud);
  pcl::transformPointCloud(cloud, cloud, transform);
  pcl::toPCLPointCloud2(cloud, mesh->cloud); 
}

Eigen::Transform<double, 3, Eigen::Isometry> particle_to_eigen_transform(const Particle &p)
{
  return Eigen::Translation3d(p.block(0,0,3,1))
    * Eigen::AngleAxis<double>(p(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxis<double>(p(4), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxis<double>(p(3), Eigen::Vector3d::UnitX());
}

int main(int argc, char **argv)
{
  char stl_name[999];
  scanf("%s",stl_name);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  pcl::io::loadPolygonFileSTL(stl_name, *mesh);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh->cloud, cloud);
  Eigen::Matrix4d trans;
  trans.setIdentity();
  trans.block(0,0,3,3)/=1000.0;
  pcl::transformPointCloud(cloud, cloud, Eigen::Transform<double,3,2>(trans));
  pcl::toPCLPointCloud2(cloud, mesh->cloud); 

  ros::init(argc,  argv, "test_client");
  actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> client("update_distribution", true);
  client.waitForServer();

  int time;
  scanf("%d",&time);
  for(int t=0;t<time;t++){
    Particle rand_particle0 = get_random_particle(), mean = get_random_particle();

    auto gripper_pose=to_PoseWithCovariance(rand_particle0, CovarianceMatrix::Zero()).pose;
    auto gripper_transform = particle_to_eigen_transform(rand_particle0);
    
    auto input_trans = gripper_transform * particle_to_eigen_transform(mean);
    pcl::PolygonMesh::Ptr input_mesh(new pcl::PolygonMesh());
    *input_mesh=*mesh;
    MeshTransform(input_mesh, input_trans);
    char input_mesh_name[999];
    sprintf(input_mesh_name, "results/input%d.stl",t);
    pcl::io::savePolygonFileSTL(input_mesh_name, *input_mesh);
    printf("Print to %s.\n", input_mesh_name);
    
    o2ac_msgs::updateDistributionGoal goal;
    goal.observation_type = goal.PLACE_OBSERVATION;
    goal.place_observation.gripper_pose = gripper_pose;
    goal.place_observation.ground_z = -0.1;
    goal.distribution = to_PoseWithCovariance(mean, CovarianceMatrix::Zero());
    client.sendGoal(goal);
    client.waitForResult();
    auto result = client.getResult();
    if(result->success){
      auto output_trans = gripper_transform * pose_to_eigen_transform(result->distribution.pose);
      pcl::PolygonMesh::Ptr output_mesh(new pcl::PolygonMesh());
      *output_mesh=*mesh;
      MeshTransform(output_mesh, output_trans);
      char output_mesh_name[999];
      sprintf(output_mesh_name, "results/output%d.stl",t);
      pcl::io::savePolygonFileSTL(output_mesh_name, *output_mesh);
      printf("Print to %s.\n", output_mesh_name);
    }
    else{
      puts("Unstable");
    }
  }
  return 0;
}
