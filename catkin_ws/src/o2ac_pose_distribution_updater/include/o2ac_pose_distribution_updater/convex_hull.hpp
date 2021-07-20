#include <Eigen/Geometry>
#include <vector>

void convex_hull_for_Eigen_Vector2d(std::vector<Eigen::Vector2d> &points,
                                    std::vector<Eigen::Vector2d> &hull);

bool check_inside_convex_hull(const Eigen::Vector2d &point,
                              const std::vector<Eigen::Vector2d> &points);

bool do_intersect_convex_hulls(const std::vector<Eigen::Vector2d> &points_0,
                               const std::vector<Eigen::Vector2d> &points_1);
