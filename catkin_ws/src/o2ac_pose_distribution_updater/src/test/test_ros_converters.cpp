#include "../estimator.cpp"
#include <geometry_msgs/PoseWithCovariance.h>
#include "../ros_converters.cpp"

int main()
{
  Particle p;
  CovarianceMatrix C;
  for(int i=0;i<6;i++){
    p(i)= i / 10.0;
    for(int j=0;j<6;j++){
      C(i,j)= i / 10.0 + j / 100.0;
    }
  }
  auto gm=to_PoseWithCovariance(p,C);
  auto np = pose_to_particle(gm.pose);
  auto nC = array_36_to_matrix_6x6(gm.covariance);
  std::cout << np << std::endl;
  std::cout << nC << std::endl;

  auto trans1 = particle_to_transform(p);
  auto trans2 =  pose_to_transform(gm.pose);
  printf("%lf %lf %lf %lf %lf %lf %lf\n",
	 trans1.getTranslation().data[0],
	 trans1.getTranslation().data[1],
	 trans1.getTranslation().data[2],
	 trans1.getQuatRotation().getW(),
	 trans1.getQuatRotation().getX(),
	 trans1.getQuatRotation().getY(),
	 trans1.getQuatRotation().getZ());
  printf("%lf %lf %lf %lf %lf %lf %lf\n",
	 trans2.getTranslation().data[0],
	 trans2.getTranslation().data[1],
	 trans2.getTranslation().data[2],
	 trans2.getQuatRotation().getW(),
	 trans2.getQuatRotation().getX(),
	 trans2.getQuatRotation().getY(),
	 trans2.getQuatRotation().getZ());
  return 0;
}

