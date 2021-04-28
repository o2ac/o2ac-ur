/*
This program generates test cases for test_client_for_touch_action

Usage:
The first argument is the name of stl file representing the gripped object
The second argument is the number 'time' of action call

This program reads the true pose of the object represented as particle from the standard input and writes 'time' lines
Each line is of the form "id x y z qx qy qz qw" where "id" is the index of the touched object (0: ground, 1: box) and other 7 values represents the pose of the gripper as cartesian coordinates and quaternion when the gripped object touches the touched object.
 */

#include "../estimator.cpp"
#include "../tools.cpp"
#include <cstdio>
#include <random>
#include <cmath>
#include <iostream>

const double pi = acos(-1.0);

namespace{
  std::random_device seed_generator;
  std::default_random_engine engine(seed_generator());
  std::uniform_real_distribution<double> dist(-2.0, 2.0);

  Particle get_random_particle()
  {
    Particle p;
    p<<dist(engine), dist(engine), -1.0 + dist(engine), pi * dist(engine), pi * dist(engine), pi * dist(engine);
    return p;
  }
}


int main(int argc, char **argv)
{
  std::string stl_file_path(argv[1]);
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> gripped_geometry(new fcl::BVHModel<fcl::OBBRSS>());
  load_BVHModel_from_stl_file(stl_file_path, gripped_geometry);

  // The parameters are same as the values in the launch file.
  std::shared_ptr<fcl::Box> ground_geometry(new fcl::Box(2.0, 2.0, 2.0));
  fcl::Vec3f ground_translation(0.0, 0.0, -1.0 + 0.01 - 0.004);
  fcl::Transform3f ground_transform(fcl::Quaternion3f(),ground_translation);
  std::shared_ptr<fcl::CollisionObject> ground_object(new fcl::CollisionObject(ground_geometry, ground_transform));

  std::shared_ptr<fcl::Box> box_geometry(new fcl::Box(0.4, 0.2, 0.097));
  fcl::Vec3f box_translation(-0.20, -0.485, 0.0485);
  fcl::Transform3f box_transform(fcl::Quaternion3f(),box_translation);
  std::shared_ptr<fcl::CollisionObject> box_object(new fcl::CollisionObject(box_geometry, box_transform));

  double distance_threshold = 0.0002;

  int time = atoi(argv[2]);

  // Input the pose of the gripped object
  Particle answer;
  double x,y,z,ex,ey,ez;
  scanf("%lf%lf%lf%lf%lf%lf",&x,&y,&z,&ex,&ey,&ez);
  answer << x,y,z,ex,ey,ez;
  fcl::Transform3f answer_trans = particle_to_transform(answer);
  
  for(int i=0;i<time;i++){
    int id = rand()%2;
    std::shared_ptr<fcl::CollisionObject> touched_object = (id ==0 ? ground_object : box_object);
    
    Particle p0, p1;
    // Find a gripper pose 'p0' such that the gripped objects intersects 'touched_object'
    while(1){
      p0 = get_random_particle();
      double distance = calculate_distance(touched_object, gripped_geometry, particle_to_transform(p0) * answer_trans);
      if(distance_threshold > distance){
	break;
      }
    }
    // Find a gripper pose 'p1' such that the gripped objects is separated from 'touched_object'
    while(1){
      p1 = get_random_particle();
      double distance = calculate_distance(touched_object, gripped_geometry, particle_to_transform(p1) * answer_trans);
      if(distance_threshold < distance){
	break;
      }
    }
    // Find a gripper pose such that the gripped objects touches 'touched_object' by binary search between 'p0' and 'p1'
    while(1){
      Particle p2 = (p0 + p1) / 2.0;
      double distance = calculate_distance(touched_object, gripped_geometry, particle_to_transform(p2) * answer_trans);
      if(distance_threshold > distance){
	p0 = p2;
	if(distance > -0.1){
	  // The gripped object is near enough to 'touched_object' but does not intersent it
	  // So the current value of 'p0' is a wanted pose
	  break;
	}
      }
      else{
	p1 = p2;
      }
    }
    // Output 'p0' as a pair of xyz coordinates and quaternion
    Eigen::Quaterniond quat = Eigen::AngleAxisd(p0(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(p0(4), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(p0(3), Eigen::Vector3d::UnitX());
    Eigen::Vector4d coeffs =  quat.coeffs();
    printf("%d %lf %lf %lf %lf %lf %lf %lf\n",id, p0(0), p0(1), p0(2), coeffs(0), coeffs(1), coeffs(2), coeffs(3));
  }
  return 0;
}
