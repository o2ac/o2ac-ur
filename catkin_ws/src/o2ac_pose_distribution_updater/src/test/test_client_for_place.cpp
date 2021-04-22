#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include "../estimator.cpp"
#include "../ros_converters.cpp"
#include "o2ac_msgs/updateDistributionAction.h"

void MeshTransform(const pcl::PolygonMesh::Ptr &mesh,
		   const Eigen::Transform<double, 3, 2> &transform)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh->cloud, cloud);
  pcl::transformPointCloud(cloud, cloud, transform);
  pcl::toPCLPointCloud2(cloud, mesh->cloud); 
}

int main(int argc, char **argv)
{
  char stl_name[99];
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
    Particle rand_particle0, mean;
    for(int i=0;i<3;i++){
      rand_particle0(i) = rand()%100/1000.0;
      mean(i) = rand()%100/1000.0;
    }
    for(int i=3;i<6;i++){
      rand_particle0(i) = rand()%100/50.0 * 3.1416;
      mean(i) = rand()%100/50.0 * 3.1416;
    }
    auto gripper_pose=to_PoseWithCovariance(rand_particle0, CovarianceMatrix::Zero()).pose;
    auto gripper_transform = particle_to_eigen_transform(rand_particle0);
    
    auto input_trans = particle_to_eigen_transform(mean) * gripper_transform;
    pcl::PolygonMesh::Ptr input_mesh(new pcl::PolygonMesh());
    *input_mesh=*mesh;
    MeshTransform(input_mesh, input_trans);
    char input_mesh_name[99];
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
      auto output_trans = pose_to_eigen_transform(result->distribution.pose) * gripper_transform;
      pcl::PolygonMesh::Ptr output_mesh(new pcl::PolygonMesh());
      *output_mesh=*mesh;
      MeshTransform(output_mesh, output_trans);
      char output_mesh_name[99];
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
