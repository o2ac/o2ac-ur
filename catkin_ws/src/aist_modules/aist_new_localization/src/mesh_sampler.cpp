// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <ros/package.h>

inline double
uniform_deviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void
randomPointTriangle(float a1, float a2, float a3,
		    float b1, float b2, float b3,
		    float c1, float c2, float c3,
		    float r1, float r2, Eigen::Vector3f& p)
{
    const auto	r1sqr = std::sqrt(r1);
    const auto	OneMinR1Sqr = (1 - r1sqr);
    const auto	OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
}

inline void
randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas,
	     double totalArea, Eigen::Vector3f& p)
{
    const auto	r = static_cast<float>(uniform_deviate(rand()) * totalArea);

    auto	low = std::lower_bound(cumulativeAreas->begin(),
				       cumulativeAreas->end(), r);
    auto	el = vtkIdType(low - cumulativeAreas->begin());

    double	A[3], B[3], C[3];
    vtkIdType	npts = 0;
    vtkIdType*	ptIds = nullptr;
    polydata->GetCellPoints(el, npts, ptIds);
    polydata->GetPoint(ptIds[0], A);
    polydata->GetPoint(ptIds[1], B);
    polydata->GetPoint(ptIds[2], C);

    const auto	r1 = static_cast<float>(uniform_deviate(rand()));
    const auto	r2 = static_cast<float>(uniform_deviate(rand()));
    randomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
			float(B[0]), float(B[1]), float(B[2]),
			float(C[0]), float(C[1]), float(C[2]), r1, r2, p);
}

void
uniform_sampling(vtkSmartPointer<vtkPolyData> polydata,
		 size_t n_samples,
		 pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out)
{
    polydata->BuildCells();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

    double		p1[3], p2[3], p3[3], totalArea = 0;
    std::vector<double>	cumulativeAreas(cells->GetNumberOfCells(), 0);
    vtkIdType		npts = 0, *ptIds = nullptr;
    size_t		cellId = 0;
    for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); cellId++)
    {
	polydata->GetPoint(ptIds[0], p1);
	polydata->GetPoint(ptIds[1], p2);
	polydata->GetPoint(ptIds[2], p3);
	totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
	cumulativeAreas[cellId] = totalArea;
    }

    cloud_out.points.resize(n_samples);
    cloud_out.width = static_cast<pcl::uint32_t>(n_samples);
    cloud_out.height = 1;

    for (size_t i = 0; i < n_samples; i++)
    {
	Eigen::Vector3f	p;
	Eigen::Vector3f	n(0, 0, 0);
	Eigen::Vector3f	c(0, 0, 0);
	randPSurface(polydata, &cumulativeAreas, totalArea, p);
	cloud_out.points[i].x = p[0];
	cloud_out.points[i].y = p[1];
	cloud_out.points[i].z = p[2];
    }
}

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

constexpr int	default_number_samples = 100000;
constexpr float	default_leaf_size = 0.01f;
constexpr float	default_scale = 1.0f;

void
printHelp(int, char** argv)
{
    print_error("Syntax is: %s input.{ply,obj} output.pcd <options>\n",
		 argv[0]);
    print_info("  where options are:\n");
    print_info("                     -n_samples X      = number of samples(default: ");
    print_value("%d", default_number_samples);
    print_info(")\n");
    print_info(
	"                     -leaf_size X  = the XYZ leaf size for the VoxelGrid -- for data reduction(default: ");
    print_value("%f", default_leaf_size);
    print_info(
	"                     -scale = scaling factor for representing in meters(default: ");
    print_value("%f", default_scale);
    print_info(")\n");
}

/* ---[ */
int
main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_sampler");
    print_info("Convert a CAD model to a point cloud using uniform sampling.\n");

    std::string package_path = ros::package::getPath("o2ac_parts_description");

    if (argc < 3)
    {
	printHelp(argc, argv);
	return -1;
    }

  // Parse command line arguments
    auto	SAMPLE_POINTS_ = default_number_samples;
    parse_argument(argc, argv, "-n_samples", SAMPLE_POINTS_);
    auto	leaf_size = default_leaf_size;
    parse_argument(argc, argv, "-leaf_size", leaf_size);
    auto	scale = default_scale;
    parse_argument(argc, argv, "-scale", scale);

  // Parse the command line arguments for .ply and PCD files
    const auto	pcd_file_indices = parse_file_extension_argument(argc, argv,
								 ".pcd");
    if (pcd_file_indices.size() != 1)
    {
	print_error("Need a single output PCD file to continue.\n");
	return -1;
    }
    const auto	ply_file_indices = parse_file_extension_argument(argc, argv,
								 ".ply");
    if (ply_file_indices.size() != 1)
    {
	print_error("Need a single input PLY file to continue.\n");
	return(-1);
    }

  // reads the PLY file
    auto		polydata1 = vtkSmartPointer<vtkPolyData>::New();
    pcl::PolygonMesh	mesh;
    pcl::io::loadPolygonFilePLY(package_path + "/meshes/"
				+ argv[ply_file_indices[0]], mesh);
    pcl::io::mesh2vtk(mesh, polydata1);

  //make sure that the polygons are triangles!
    const auto	triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(polydata1);
    triangleFilter->Update();

    const auto	triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
    triangleMapper->Update();
    polydata1 = triangleMapper->GetInput();

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
	cloud_1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    uniform_sampling(polydata1, SAMPLE_POINTS_, *cloud_1);

  // Voxelgrid
    VoxelGrid<PointXYZRGBNormal> grid_;
    grid_.setInputCloud(cloud_1);
    grid_.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
	voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    grid_.filter(*voxel_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
	cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  // Strip uninitialized normals and colors from cloud:
    pcl::copyPointCloud(*voxel_cloud, *cloud_xyz);

  // Scale cloud
    Eigen::Matrix4f	transform = Eigen::Matrix4f::Zero();
    transform(0, 0) = scale;
    transform(1, 1) = scale;
    transform(2, 2) = scale;
    transform(3, 3) = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr
	cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_xyz, *cloud_out, transform);

  // Save cloud
    savePCDFileASCII(package_path + "/meshes/" + argv[pcd_file_indices[0]],
		     *cloud_out);
}
