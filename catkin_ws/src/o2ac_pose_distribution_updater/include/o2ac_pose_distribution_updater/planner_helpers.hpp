#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>

#include "o2ac_pose_distribution_updater/conversions.hpp"
#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <vector>

void calculate_place_candidates(
    const std::vector<Eigen::Vector3d> &vertices,
    const Eigen::Vector3d &center_of_gravity,
    std::vector<Eigen::Hyperplane<double, 3>> &candidates);
