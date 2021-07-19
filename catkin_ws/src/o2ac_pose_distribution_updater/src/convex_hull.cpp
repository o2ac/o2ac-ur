#include "o2ac_pose_distribution_updater/convex_hull.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <algorithm>

void convex_hull_for_Eigen_Vector2d(std::vector<Eigen::Vector2d> &points,
                                    std::vector<Eigen::Vector2d> &hull) {
  // calculate the convex hull using CGAL

  using CGAL_Point_2 =
      CGAL::Exact_predicates_inexact_constructions_kernel::Point_2;

  // convert points to CGAL type
  std::vector<CGAL_Point_2> CGAL_points, CGAL_hull;
  std::transform(points.begin(), points.end(), std::back_inserter(CGAL_points),
                 [](const Eigen::Vector2d &point) {
                   return CGAL_Point_2(point(0), point(1));
                 });

  // calculate the convex hull
  CGAL::convex_hull_2(CGAL_points.begin(), CGAL_points.end(),
                      std::back_inserter(CGAL_hull));

  // convert hull to Eigen::Vector2d
  std::transform(CGAL_hull.begin(), CGAL_hull.end(), std::back_inserter(hull),
                 [](const CGAL_Point_2 &point) {
                   Eigen::Vector2d vector2d;
                   vector2d << point.x(), point.y();
                   return vector2d;
                 });
}
