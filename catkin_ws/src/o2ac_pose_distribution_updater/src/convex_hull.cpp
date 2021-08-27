#include "o2ac_pose_distribution_updater/convex_hull.hpp"

/*
CGAL convex_hull_2 is used since boost::geometry convex_hull is not robust
*/

#include "o2ac_pose_distribution_updater/conversions.hpp"
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/convex_hull_2.h>
#include <algorithm>
#include <boost/geometry/geometries/polygon.hpp>

using CGAL_Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CGAL_Point_2 = CGAL_Kernel::Point_2;
using CGAL_Polygon_2 = CGAL::Polygon_2<CGAL_Kernel>;

CGAL_Point_2 point_2_Eigen_to_CGAL(const Eigen::Vector2d &point) {
  return CGAL_Point_2(point(0), point(1));
}

Eigen::Vector2d point_2_CGAL_to_Eigen(const CGAL_Point_2 &point) {
  Eigen::Vector2d vector2d;
  vector2d << CGAL::to_double(point.x()), CGAL::to_double(point.y());
  return vector2d;
}

void convex_hull_for_Eigen_Vector2d(std::vector<Eigen::Vector2d> &points,
                                    std::vector<Eigen::Vector2d> &hull) {
  // calculate the convex hull using CGAL

  // convert points to CGAL type
  std::vector<CGAL_Point_2> CGAL_points, CGAL_hull;
  std::transform(points.begin(), points.end(), std::back_inserter(CGAL_points),
                 point_2_Eigen_to_CGAL);

  // calculate the convex hull
  CGAL::convex_hull_2(CGAL_points.begin(), CGAL_points.end(),
                      std::back_inserter(CGAL_hull));

  // convert hull to Eigen::Vector2d
  std::transform(CGAL_hull.begin(), CGAL_hull.end(), std::back_inserter(hull),
                 point_2_CGAL_to_Eigen);
}

bool check_inside_convex_hull(const Eigen::Vector2d &point,
                              const std::vector<Eigen::Vector2d> &points) {
  // convert points to CGAL type
  std::vector<CGAL_Point_2> CGAL_points, CGAL_hull;
  std::transform(points.begin(), points.end(), std::back_inserter(CGAL_points),
                 point_2_Eigen_to_CGAL);

  // calculate the convex hull
  CGAL::convex_hull_2(CGAL_points.begin(), CGAL_points.end(),
                      std::back_inserter(CGAL_hull));

  // check inside
  auto result =
      CGAL::bounded_side_2(CGAL_hull.begin(), CGAL_hull.end(),
                           point_2_Eigen_to_CGAL(point), CGAL_Kernel());
  return result == CGAL::ON_BOUNDED_SIDE;
}

void convex_hull_to_boost(
    const std::vector<Eigen::Vector2d> &points,
    boost::geometry::model::ring<Eigen::Vector2d, false> &boost_hull) {
  // convert points to CGAL type
  std::vector<CGAL_Point_2> CGAL_points, CGAL_hull;
  std::transform(points.begin(), points.end(), std::back_inserter(CGAL_points),
                 point_2_Eigen_to_CGAL);

  // calculate the convex hull
  CGAL::convex_hull_2(CGAL_points.begin(), CGAL_points.end(),
                      std::back_inserter(CGAL_hull));

  // covert to boost ring
  for (auto &vertex : CGAL_hull) {
    boost::geometry::append(boost_hull, point_2_CGAL_to_Eigen(vertex));
  }
  boost::geometry::append(boost_hull, point_2_CGAL_to_Eigen(CGAL_hull[0]));
}

bool do_intersect_convex_hulls(const std::vector<Eigen::Vector2d> &points_0,
                               const std::vector<Eigen::Vector2d> &points_1) {
  boost::geometry::model::ring<Eigen::Vector2d, false> boost_hull_0,
      boost_hull_1;
  convex_hull_to_boost(points_0, boost_hull_0);
  convex_hull_to_boost(points_1, boost_hull_1);
  return boost::geometry::intersects(boost_hull_0, boost_hull_1);
}
