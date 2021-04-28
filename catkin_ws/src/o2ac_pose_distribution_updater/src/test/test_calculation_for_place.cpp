/*
This program reads coordinates of a tetrahedron from standard input
and writes its coordinates after placing it to the ground "z=0.0" to standard output
 */

#include "../estimator.cpp"

#include <cstdio>

int main()
{
  std::vector<Eigen::Vector3d>  tetra(4);
  for(int i=0; i<4; i++){
    double x,y,z;
    scanf("%lf%lf%lf",&x,&y,&z);
    tetra[i] << x,y,z;
  }
  auto cog = (tetra[0]+tetra[1]+tetra[2]+tetra[3])/4.0;
  int v1,v2,v3;
  Eigen::Quaterniond rot;
  bool stability;
  try{
    find_three_points(tetra, cog, v1, v2, v3, rot, stability);
  }
  catch(std::runtime_error &e){
    std::cerr << e.what() << std::endl;
    return 0;
  } 
  printf("%d %d %d\n",v1,v2,v3);
  Particle mean, new_mean;
  CovarianceMatrix covariance, new_covariance;
  mean.setZero();
  covariance.setZero();
  place_update_distribution(mean, covariance, cog, tetra[v1],
			    tetra[v2], tetra[v3], 0.0, Eigen::Transform<double, 3, Eigen::Isometry>::Identity(), 
			    new_mean, new_covariance);
  auto place_trans = particle_to_eigen_transform(new_mean);
  for(int i=0; i<4; i++){
    tetra[i] = place_trans * tetra[i];
    printf("%lf %lf %lf\n",tetra[i][0], tetra[i][1], tetra[i][2]);
  }
  return 0;
}
