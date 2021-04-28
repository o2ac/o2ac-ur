/*
The functions associated to place action
 */
#include <Eigen/Geometry>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <unsupported/Eigen/AutoDiff>
#include "conversions.cpp"

int argmin(const std::vector<double> &vec){
  return std::distance(vec.begin(), std::min_element(vec.begin(), vec.end()));
}

void find_three_points(const std::vector<Eigen::Vector3d> &current_vertices,
		       const Eigen::Vector3d &current_cog,
		       int &first_point_id,
		       int &second_point_id,
		       int &third_point_id,
		       Eigen::Quaterniond &rotation,
		       bool &stability)
{
  // Given the coordinates of vertices and center of gravity, find the three points touching the ground after placing
  // The object rotation occured by placing is stored to 'rotation'
  // Stabliity after placing is checked and stored to 'stability'
  
  const double INF=1e9, EPS=1e-6;
  
  int num_of_vertices = current_vertices.size();

  // The first point touching the ground is the vertice with the minimum z coordinate
  std::vector<double> current_vertices_z(num_of_vertices);
  for(int i=0;i<num_of_vertices;i++){
    current_vertices_z[i] = current_vertices[i](2);
  }
  first_point_id = argmin(current_vertices_z);

  // After the first point touched the ground, the object rotates with an axis, which is orthogonal to both z axis and the line connecting the first point and the center of gravity
  // The direction of rotation is the one such that the center of gravity approached the ground
  Eigen::Vector3d first_axis = (current_vertices[first_point_id] - current_cog).cross(Eigen::Vector3d::UnitZ());
  if(first_axis.norm()<EPS){
    throw std::runtime_error("Balanced at the first rotation");
  }
  first_axis=first_axis.normalized();
  // When another point touches the ground, the rotation stops
  // So the second touching point is the vertices with the minimum rotation angle to touch the ground
  std::vector<double> first_angles(num_of_vertices);
  for(int i=0;i<num_of_vertices;i++){
    auto v1v2 = current_vertices[i] - current_vertices[first_point_id];
    first_angles[i]=(i==first_point_id?INF:
		     std::atan2(v1v2(2), first_axis(1) * v1v2(0) - first_axis(0) * v1v2(1)));
  }
  second_point_id = argmin(first_angles);

  // calculate the coordinates of vertices and the center of gravity after the first rotation
  auto first_rotation = Eigen::AngleAxisd(first_angles[second_point_id], first_axis);
  auto rotated_cog = first_rotation * current_cog;
  std::vector<Eigen::Vector3d> rotated_vertices(num_of_vertices);
  for(int i=0; i<num_of_vertices; i++){
    rotated_vertices[i] = first_rotation * current_vertices[i];
  }

  // After the second point touched the ground, the object rotates with an axis, which is the line connecting the first touching point and the second touching point
  auto second_axis = (rotated_vertices[second_point_id] - rotated_vertices[first_point_id]).normalized();
  // The direction of rotation is the one such that the center of gravity approached the ground
  double direction=(rotated_cog-rotated_vertices[first_point_id]).cross(second_axis)(2);
  if(std::abs(direction) < EPS){
    throw std::runtime_error("Balanced at the second rotation");
  }
  if(direction < 0.0){
    second_axis = -second_axis;
  }
  // When another point touches the ground, the rotation stops
  // So the third touching point is the vertices with the minimum rotation angle to touch the ground
  std::vector<double> second_angles(num_of_vertices);
  for(int i=0;i<num_of_vertices;i++){
    auto v1v3 = rotated_vertices[i] - rotated_vertices[first_point_id];
    second_angles[i]=(i==first_point_id || i==second_point_id ?INF:
		      std::atan2(v1v3(2), second_axis(1) * v1v3(0) - second_axis(0) * v1v3(1)));
  }
  third_point_id = argmin(second_angles);

  //calculate the total rotation
  auto second_rotation = Eigen::AngleAxisd(second_angles[third_point_id], second_axis);
  rotation = second_rotation * first_rotation;


  // stability check
  
  // calculate the coordinates after the second rotation
  auto final_cog = second_rotation * rotated_cog;
  std::vector<Eigen::Vector3d> final_vertices(num_of_vertices);
  for(int i=0;i<num_of_vertices;i++){
    final_vertices[i] = second_rotation * rotated_vertices[i];
  }
    
  // calculate the convex hull of the vertices touching the ground
  double min_z = final_vertices[first_point_id](2);
    
  namespace bg=boost::geometry;
  using bg_2d_point=bg::model::d2::point_xy<double>;
  bg::model::multi_point<bg_2d_point> points_on_ground;
  
  for(int i=0;i<num_of_vertices;i++){
    auto ver = final_vertices[i];
    if(ver(2) <= min_z + EPS){
      bg::append(points_on_ground, bg_2d_point(ver(0), ver(1)));
    }
  }

  bg::model::polygon<bg_2d_point> hull;
  bg::convex_hull(points_on_ground, hull);
  
  // If the center of geometry projected to ground is in the convex hull, the object is stable
  bg_2d_point projected_cog(final_cog(0), final_cog(1));

  stability = bg::within(projected_cog, hull);
}

namespace{
  
  // Calculate the function from the pose before placing to the pose after placing and its Jacobian.
  // To use AutoDiff in Eigen-unsupported, the function is calculated in the class "calculate_ptc".

  Eigen::Vector3d g,v1,v2,v3; // the coordinates of center of gravity, first touching point, second touching point and third touching point
  double gr_z; // the z-coordinate of the ground
  Eigen::Transform<double, 3, Eigen::Isometry> gr_trans; // The gripper transform

  class calculate_ptc {
    
  public:

    // Neede by Eigen AutoDiff
    enum
    {
      InputsAtCompileTime = 6,
      ValuesAtCompileTime = 6
    };
     
    // Also needed by Eigen AutoDiff
    typedef Eigen::Matrix<double, 6, 1> InputType;
    typedef Eigen::Matrix<double, 6, 1> ValueType;
   
    // The Vector function from the particle representing the pose before placing to the particle representing the pose after placing.
    // To use AutoDiff, the type of coordinates is templated by typename "T".
    template <typename T>
    void operator()
    (
      const Eigen::Matrix<T, 6, 1>& cur_ptc, 
      Eigen::Matrix<T, 6, 1>* result_ptc
    ) const
    {
      using point = Eigen::Matrix<T, 3, 1>;

      // calculate the coordinates when the pose is represented by particle 'cur_ptc'
      auto gr_trans_T = (Eigen::Transform<T, 3, Eigen::Isometry>)gr_trans; 
      auto cur_trans = gr_trans_T
	* Eigen::Translation<T, 3>(cur_ptc.block(0,0,3,1))
	* Eigen::AngleAxis<T>(cur_ptc(5), point::UnitZ())
	* Eigen::AngleAxis<T>(cur_ptc(4), point::UnitY())
	* Eigen::AngleAxis<T>(cur_ptc(3), point::UnitX());
      point cur_g  = cur_trans * (point)g;
      point cur_v1 = cur_trans * (point)v1;
      point cur_v2 = cur_trans * (point)v2;
      point cur_v3 = cur_trans * (point)v3;

      // calculate the first rotation
      point v1v2 = cur_v2 - cur_v1;
      point first_axis  = (cur_v1 - cur_g).cross(point::UnitZ()).normalized();
      T first_angle = atan2(v1v2(2), first_axis(1) * v1v2(0) - first_axis(0) * v1v2(1));
      auto first_rot = Eigen::AngleAxis<T>(first_angle, first_axis);
  
      // calculate the coordinates after first rotation
      point rd_g  = first_rot * cur_g;
      point rd_v1 = first_rot * cur_v1;
      point rd_v2 = first_rot * cur_v2;
      point rd_v3 = first_rot * cur_v3;
  
      // calculate the second rotation
      point v1v3 = rd_v3 - rd_v1;
      point second_axis = (rd_v2 - rd_v1).normalized();
      if(second_axis.cross(point::UnitZ()).dot(rd_g - rd_v1) < 0){
	second_axis = -second_axis;
      }
      T second_angle = atan2(v1v3(2), second_axis(1) * v1v3(0) - second_axis(0) * v1v3(1));
      auto second_rot(Eigen::AngleAxis<T>(second_angle, second_axis));

      // calculate the coordinates after second rotation
      auto final_g  = second_rot * rd_g;
      auto final_v1 = second_rot * rd_v1;
      auto final_v2 = second_rot * rd_v2;
      auto final_v3 = second_rot * rd_v3;
      // The translation is occured to hold the physical restraints
      point final_trans =
	(cur_g(0) - final_g(0))  * (point::UnitX()) // The x-coordinate of the center of gravity is not changed
	+ (cur_g(1) - final_g(1))  * (point::UnitY()) // The y-coordinate of the center of gravity is not changed
	+ (gr_z - final_v1(2)) * (point::UnitZ());    // The z-coordinate of the vertices touching the ground is that of the ground
      // calculate the pose after placing
      auto result_trans = gr_trans_T.inverse() * Eigen::Translation<T, 3>(final_trans) * second_rot * first_rot * cur_trans;
      // convert it to Particle
      result_ptc->block(0,0,3,1) = result_trans.translation();
      Eigen::Quaternion<T> result_rot(result_trans.rotation());
      T roll, pitch, yaw;
      quaternion_to_RPY(result_rot.w(), result_rot.x(), result_rot.y(), result_rot.z(), roll, pitch, yaw);
      result_ptc->block(3,0,3,1) << roll, pitch, yaw;
    }
  };
}

void place_update_distribution(const Particle &old_mean, const CovarianceMatrix &old_covariance,
			       const Eigen::Vector3d &cog, const Eigen::Vector3d &first_point,
			       const Eigen::Vector3d &second_point, const Eigen::Vector3d &third_point, const double &ground_z,
			       const Eigen::Transform<double, 3, Eigen::Isometry> gripper_transform,
			       Particle &new_mean, CovarianceMatrix new_covariance)
{
  g=cog, v1=first_point, v2=second_point, v3=third_point, gr_z=ground_z;
  gr_trans = gripper_transform;
  // Calculate the particle after placing and its Jacobian
  Eigen::AutoDiffJacobian<calculate_ptc> calculate_ptc_AD;
  // By Eigen AutoDiff, calculate_ptc_AD automatically calculates the operation of calculate_ptc and its Jacobian
  CovarianceMatrix Jacobian;
  calculate_ptc_AD(old_mean, &new_mean, &Jacobian);

  // The covariance of the function value is calculated by the covariance of the argument and Jacobian.
  new_covariance = Jacobian * old_covariance * Jacobian.transpose();
}
