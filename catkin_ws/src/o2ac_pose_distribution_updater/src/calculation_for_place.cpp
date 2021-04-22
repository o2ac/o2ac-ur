#include <Eigen/Geometry>
#include <unsupported/Eigen/AutoDiff>
#include "conversions.cpp"

int argmin(const std::vector<double> &vec){
  return std::distance(vec.begin(), std::min_element(vec.begin(), vec.end()));
}


double ccw(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c)
{
  return (b-a).cross(c-a)(2);
}

void find_three_points(const std::vector<Eigen::Vector3d> &current_vertices,
		       const Eigen::Vector3d &current_cog,
		       int &first_point_id,
		       int &second_point_id,
		       int &third_point_id,
		       Eigen::Quaterniond &rotation)
{
  const double INF=1e9, EPS=1e-3;
  
  int num_of_vertices = current_vertices.size();
  
  std::vector<double> current_vertices_z(num_of_vertices);
  for(int i=0;i<num_of_vertices;i++){
    current_vertices_z[i] = current_vertices[i](2);
  }
  first_point_id = argmin(current_vertices_z);
  Eigen::Vector3d first_axis = (current_vertices[first_point_id] - current_cog).cross(Eigen::Vector3d::UnitZ()).normalized();
  std::vector<double> first_angles(num_of_vertices);
  for(int i=0;i<num_of_vertices;i++){
    auto v1v2 = current_vertices[i] - current_vertices[first_point_id];
    first_angles[i]=(i==first_point_id?INF:
		     std::atan2(v1v2(2), first_axis(1) * v1v2(0) - first_axis(0) * v1v2(1)));
  }
  second_point_id = argmin(first_angles);
  auto first_rotation = Eigen::AngleAxisd(first_angles[second_point_id], first_axis);
  auto rotated_cog = first_rotation * current_cog;
  std::vector<Eigen::Vector3d> rotated_vertices(num_of_vertices);
  for(int i=0; i<num_of_vertices; i++){
    rotated_vertices[i] = first_rotation * current_vertices[i];
  }
  auto second_axis = (rotated_vertices[second_point_id] - rotated_vertices[first_point_id]).normalized();
  if(second_axis.cross(Eigen::Vector3d::UnitZ()).dot(rotated_cog - rotated_vertices[first_point_id]) < 0){
    second_axis = -second_axis;
  }
  std::vector<double> second_angles(num_of_vertices);
  for(int i=0;i<num_of_vertices;i++){
    auto v1v3 = rotated_vertices[i] - rotated_vertices[first_point_id];
    second_angles[i]=(i==first_point_id || i==second_point_id ?INF:
		      std::atan2(v1v3(2), second_axis(1) * v1v3(0) - second_axis(0) * v1v3(1)));
  }
  third_point_id = argmin(second_angles);
  auto second_rotation = Eigen::AngleAxisd(second_angles[third_point_id], second_axis);
  rotation = second_rotation * first_rotation;
}

namespace{

  Eigen::Vector3d g,v1,v2,v3;
  double gr_z;
  
  class calculate_ptc {
    
  public:
    
    enum
    {
      InputsAtCompileTime = 6,
      ValuesAtCompileTime = 6
    };
     
    // Also needed by Eigen
    typedef Eigen::Matrix<double, 6, 1> InputType;
    typedef Eigen::Matrix<double, 6, 1> ValueType;
   
    // Vector function
    template <typename T>
    void operator()
    (
      const Eigen::Matrix<T, 6, 1>& cur_ptc, 
      Eigen::Matrix<T, 6, 1>* result_ptc
    ) const
    {
      using point = Eigen::Matrix<T, 3, 1>;
      
      point cur_trans = cur_ptc.block(0,0,3,1);
      auto cur_rot = Eigen::AngleAxis<T>(cur_ptc(5), point::UnitZ())
	* Eigen::AngleAxis<T>(cur_ptc(4), point::UnitY())
	* Eigen::AngleAxis<T>(cur_ptc(3), point::UnitX());
      point cur_g  = cur_rot._transformVector((point)g) + cur_trans;
      point cur_v1 = cur_rot._transformVector((point)v1) + cur_trans;
      point cur_v2 = cur_rot._transformVector((point)v2) + cur_trans;
      point cur_v3 = cur_rot._transformVector((point)v3) + cur_trans;
  
      point v1v2 = cur_v2 - cur_v1;
      point first_axis  = (cur_v1 - cur_g).cross(point::UnitZ()).normalized();
      T first_angle = atan2(v1v2(2), first_axis(1) * v1v2(0) - first_axis(0) * v1v2(1));
  
      Eigen::Quaternion<T> first_rot(Eigen::AngleAxis<T>(first_angle, first_axis));
  
      point rd_g  = first_rot._transformVector(cur_g);
      point rd_v1 = first_rot._transformVector(cur_v1);
      point rd_v2 = first_rot._transformVector(cur_v2);
      point rd_v3 = first_rot._transformVector(cur_v3);
  
      point v1v3 = rd_v3 - rd_v1;
      point second_axis = (rd_v2 - rd_v1).normalized();
      if(second_axis.cross(point::UnitZ()).dot(rd_g - rd_v1) < 0){
	second_axis = -second_axis;
      }
      T second_angle = atan2(v1v3(2), second_axis(1) * v1v3(0) - second_axis(0) * v1v3(1));
      Eigen::Quaternion<T> second_rot(Eigen::AngleAxis<T>(second_angle, second_axis));
  
      auto place_rot = second_rot * first_rot, total_rot  = place_rot * cur_rot;
      auto final_g  = second_rot._transformVector(rd_g);
      auto final_v1 = second_rot._transformVector(rd_v1);
      auto final_v2 = second_rot._transformVector(rd_v2);
      auto final_v3 = second_rot._transformVector(rd_v3);
      point total_trans = place_rot._transformVector(cur_trans)
	+ (cur_g(0) - final_g(0))  * (point::UnitX())
	+ (cur_g(1) - final_g(1))  * (point::UnitY())
	+ (gr_z - final_v1(2)) * (point::UnitZ());
      result_ptc->block(0,0,3,1) = total_trans;
      T roll, pitch, yaw;
      quaternion_to_RPY(total_rot.w(), total_rot.x(), total_rot.y(), total_rot.z(), roll, pitch, yaw);
      result_ptc->block(3,0,3,1) << roll, pitch, yaw;
    }
  };
}

void place_update_distribution(const Particle &old_mean, const CovarianceMatrix &old_covariance,
			       const Eigen::Vector3d &cog, const Eigen::Vector3d &first_point,
			       const Eigen::Vector3d &second_point, const Eigen::Vector3d &third_point, const double &ground_z,
			       Particle &new_mean, CovarianceMatrix new_covariance)
{
  g=cog, v1=first_point, v2=second_point, v3=third_point, gr_z=ground_z;
  Eigen::AutoDiffJacobian<calculate_ptc> calculate_ptc_AD;
  CovarianceMatrix Jacobian;
  calculate_ptc_AD(old_mean, &new_mean, &Jacobian);
  new_covariance = Jacobian * old_covariance * Jacobian.transpose();
}
