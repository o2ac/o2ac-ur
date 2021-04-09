#include "../estimator.cpp"
#include "../tools.cpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

const double pi = acos(-1.0);

  



void treat_trial(std::string stl_file_path, double ground_plane_offset, std::string exp_file_path)
{
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> gripped_geometry(new fcl::BVHModel<fcl::OBBRSS>());
  int ret=load_BVHModel_from_stl_file(stl_file_path, gripped_geometry);
  std::shared_ptr<fcl::Box> ground_geometry(new fcl::Box(2.0, 2.0, 2.0));
  fcl::Vec3f ground_translation(0.0, 0.0, -1.0 + 0.01 - ground_plane_offset);
  fcl::Transform3f ground_transform(fcl::Quaternion3f(),ground_translation);
  std::shared_ptr<fcl::CollisionObject> ground_object(new fcl::CollisionObject(ground_geometry, ground_transform));
  std::shared_ptr<fcl::Box> box_geometry(new fcl::Box(0.4, 0.2, 0.097));
  fcl::Vec3f box_translation(-0.20, -0.485, 0.0485);
  fcl::Transform3f box_transform(fcl::Quaternion3f(),box_translation);
  std::shared_ptr<fcl::CollisionObject> box_object(new fcl::CollisionObject(box_geometry, box_transform));

  
  double dth = 0.002;
  

  int number_of_particles = 10000;
  Particle mean;
  mean.setZero();

  double var_xyz = 0.001, var_ang = pi * (0.5/180.0);
  CovarianceMatrix covariance;
  covariance.setZero();
  covariance.diagonal() << var_xyz, var_xyz, var_xyz, var_ang, var_ang, var_ang;

  Particle noise_variance;
  noise_variance << var_xyz, var_xyz, var_xyz, var_ang, var_ang, var_ang;

  PoseEstimator estimator(gripped_geometry,
			  std::vector<std::shared_ptr<fcl::CollisionObject>>{ground_object, box_object},
			  dth, noise_variance, number_of_particles);

  FILE *exp_file=fopen(exp_file_path.c_str(), "r");
  while(1){
    int id;
    double x, y, z, qa, qb, qc, qd;
    if(fscanf(exp_file, "%d%lf%lf%lf%lf%lf%lf%lf\n",&id,&x,&y,&z,&qa,&qb,&qc,&qd) == EOF){
      break;
    }
    fcl::Transform3f gripper_transform(fcl::Quaternion3f(qd, qa, qb, qc), fcl::Vec3f(x,y,z));
    //cerr << calculate_distance(id ==0 ? ground_object : box_object, gripped_geometry, gripper_transform) << std::endl;
    cerr << calculate_distance(id ==0 ? ground_object : box_object, gripped_geometry, particle_to_transform(mean) * gripper_transform) << std::endl;
    estimator.touched_step(id, gripper_transform, mean, covariance, mean, covariance);
    cout << mean << '\n';
    cout << covariance << '\n';
  }
  fclose(exp_file);
}

int main(int argc, char **argv)
{
  treat_trial(std::string(argv[1]), 0.004, std::string(argv[2]));
}
