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
  find_three_points(tetra, cog, v1, v2, v3);
  printf("%d %d %d\n",v1,v2,v3);
  Particle mean;
  CovarianceMatrix covariance;
  mean.setZero();
  covariance.setZero();
  place_update_distribution(mean, covariance, cog, tetra[v1],
			    tetra[v2], tetra[v3], 0.0,
			    mean, covariance);
  auto place_trans = to_eigen_transform(particle_to_transform(mean));
  for(int i=0; i<4; i++){
    tetra[i] = place_trans * tetra[i];
    printf("%lf %lf %lf\n",tetra[i][0], tetra[i][1], tetra[i][2]);
  }
  return 0;
}
