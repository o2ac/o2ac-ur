#include "o2ac_pose_distribution_updater/planner_helpers.hpp"
#include "o2ac_pose_distribution_updater/random_particle.hpp"

namespace {
double EPS = 1e-9;
};

using CGAL_kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

CovarianceMatrix transform_covariance(const Eigen::Isometry3d &transform,
                                      const CovarianceMatrix &covariance) {
  CovarianceMatrix AD_trasnform = Adjoint<double>(transform);
  return AD_trasnform * covariance * AD_trasnform.transpose();
}

Eigen::Vector3d vector_3_cgal_to_eigen(const CGAL_kernel::Point_3 &v) {
  Eigen::Vector3d eigen_v;
  eigen_v << v.x(), v.y(), v.z();
  return eigen_v;
}

CGAL_kernel::Point_3 vector_3_eigen_to_cgal(const Eigen::Vector3d &v) {
  return CGAL_kernel::Point_3(v[0], v[1], v[2]);
}

void facet_cgal_to_eigen(const CGAL::Polyhedron_3<CGAL_kernel>::Facet &facet,
                         std::vector<Eigen::Vector3d> &eigen_facet) {
  int degree = facet.facet_degree();
  eigen_facet.resize(degree);
  auto halfedge_handle = facet.halfedge();
  for (int i = 0; i < degree; i++) {
    eigen_facet[i] = vector_3_cgal_to_eigen(halfedge_handle->vertex()->point());
    halfedge_handle = halfedge_handle->next();
  }
}

void calculate_place_candidates(
    const std::vector<Eigen::Vector3d> &vertices,
    const Eigen::Vector3d &center_of_gravity,
    std::vector<Eigen::Hyperplane<double, 3>> &candidates) {

  // compute convex hull by CGAL
  std::vector<CGAL_kernel::Point_3> vertices_cgal;
  std::transform(vertices.begin(), vertices.end(),
                 std::back_inserter(vertices_cgal), vector_3_eigen_to_cgal);
  CGAL::Polyhedron_3<CGAL_kernel> convex_hull;
  CGAL::convex_hull_3(vertices_cgal.begin(), vertices_cgal.end(), convex_hull);

  // check all facets

  // To grouping facets with the same normal vector, sort facets by its inner
  // products with a random vector

  Eigen::Vector3d random_vector = get_UND_Vector3d();

  std::vector<std::vector<Eigen::Vector3d>>
      facets; // array of arrays of vertices on the facet
  std::vector<std::pair<double, int>>
      sort_keys; // array of pairs of inner products of normals of facets and he
                 // random vector (key to sort facets) and facet ids.

  for (auto facet_iterator = convex_hull.facets_begin();
       facet_iterator != convex_hull.facets_end(); facet_iterator++) {
    // reconvert to eigen
    std::vector<Eigen::Vector3d> facet;
    facet_cgal_to_eigen(*facet_iterator, facet);

    Eigen::Vector3d normal =
        (facet[1] - facet[0]).cross(facet[2] - facet[0]).normalized();

    int id = facets.size();
    facets.push_back(std::move(facet));
    sort_keys.push_back(std::make_pair(random_vector.dot(normal), id));
  }

  // sort facets ids
  std::sort(sort_keys.begin(), sort_keys.end());

  for (int t = 0; t < sort_keys.size();) {
    // collect vertices of facets with the same normal vectors
    std::vector<Eigen::Vector3d> facet = std::move(facets[sort_keys[t].second]);
    double key_bound = sort_keys[t].first + EPS;
    while (++t < sort_keys.size() && sort_keys[t].first < key_bound) {
      int id = sort_keys[t].second;
      facet.insert(facet.end(), std::make_move_iterator(facets[id].begin()),
                   std::make_move_iterator(facets[id].end()));
    }

    Eigen::Vector3d normal =
        (facet[1] - facet[0]).cross(facet[2] - facet[0]).normalized();
    Eigen::Vector3d axis = normal.cross(-Eigen::Vector3d::UnitZ());
    axis = (axis.norm() != 0.0 ? axis.normalized() : Eigen::Vector3d::UnitX());
    double angle =
        atan2(sqrt(pow(normal[0], 2) + pow(normal[1], 2)), -normal[2]);
    Eigen::AngleAxisd rotation(angle, axis);
    Eigen::Vector2d projected_center = (rotation * center_of_gravity).head<2>();

    namespace bg = boost::geometry;
    bg::model::multi_point<Eigen::Vector2d> projected_points;
    for (auto &vertex : facet) {
      bg::append(projected_points,
                 (Eigen::Vector2d)(rotation * vertex).head<2>());
    }

    static const bool clock_wise = false;
    bg::model::ring<Eigen::Vector2d, clock_wise> hull;
    bg::convex_hull(projected_points, hull);

    if (bg::within(projected_center, hull)) {
      candidates.push_back(Eigen::Hyperplane<double, 3>(normal, facet[0]));
    }
  }
}
