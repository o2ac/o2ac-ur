/*!
* \file		Detector.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using both intensity and depth images
*/
#include <iostream>
#include <cstdint>

#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

#include <aruco_ros/aruco_ros_utils.h>

#include "Detector.h"
#include "Plane.h"
#include "Similarity.h"
#include "Rigidity.h"

namespace aist_aruco_ros
{
/************************************************************************
*  global functions							*
************************************************************************/
// The original aruco_ros::rosCameraInfo2ArucoCamParams() has a bug
// that the cameraMatrix is not correctly setup when using unrectified
// images. So, we use our own implementation.
static aruco::CameraParameters
rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& camera_info,
			     bool useRectifiedImages)
{
    cv::Mat	cameraMatrix(3, 4, CV_64FC1);
    cv::Mat	distorsionCoeff(4, 1, CV_64FC1);
    cv::Size	size(camera_info.width, camera_info.height);
    
    cameraMatrix.setTo(0);
    distorsionCoeff.setTo(0);
    
    if (useRectifiedImages)
    {
	cameraMatrix.at<double>(0, 0) = camera_info.P[0];
	cameraMatrix.at<double>(0, 1) = camera_info.P[1];
	cameraMatrix.at<double>(0, 2) = camera_info.P[2];
	cameraMatrix.at<double>(0, 3) = camera_info.P[3];
	cameraMatrix.at<double>(1, 0) = camera_info.P[4];
	cameraMatrix.at<double>(1, 1) = camera_info.P[5];
	cameraMatrix.at<double>(1, 2) = camera_info.P[6];
	cameraMatrix.at<double>(1, 3) = camera_info.P[7];
	cameraMatrix.at<double>(2, 0) = camera_info.P[8];
	cameraMatrix.at<double>(2, 1) = camera_info.P[9];
	cameraMatrix.at<double>(2, 2) = camera_info.P[10];
	cameraMatrix.at<double>(2, 3) = camera_info.P[11];
    }
    else
    {
	cameraMatrix.at<double>(0, 0) = camera_info.K[0];
	cameraMatrix.at<double>(0, 1) = camera_info.K[1];
	cameraMatrix.at<double>(0, 2) = camera_info.K[2];
	cameraMatrix.at<double>(1, 0) = camera_info.K[3];
	cameraMatrix.at<double>(1, 1) = camera_info.K[4];
	cameraMatrix.at<double>(1, 2) = camera_info.K[5];
	cameraMatrix.at<double>(2, 0) = camera_info.K[6];
	cameraMatrix.at<double>(2, 1) = camera_info.K[7];
	cameraMatrix.at<double>(2, 2) = camera_info.K[8];

	const auto	n = std::min(int(camera_info.D.size()), 4);
	for (int i = 0; i < n; ++i)
	    distorsionCoeff.at<double>(i, 0) = camera_info.D[i];
    }

    return {cameraMatrix, distorsionCoeff, size};
}
    
template <class T> inline T
val(const sensor_msgs::Image& image_msg, int u, int v)
{
    using namespace	sensor_msgs;

    if (image_msg.encoding == image_encodings::TYPE_16UC1)
    	return T(0.001) * *reinterpret_cast<const uint16_t*>(
    				image_msg.data.data() + v*image_msg.step
    						      + u*sizeof(uint16_t));
    else
	return *reinterpret_cast<const T*>(image_msg.data.data()
					   + v*image_msg.step + u*sizeof(T));
}

/************************************************************************
*  class Detector							*
************************************************************************/
Detector::Detector(const ros::NodeHandle& nh)
    :_nh(nh),
     _tfListener(),
     _tfBroadcaster(),
     _marker_frame("marker_frame"),
     _camera_frame(""),
     _reference_frame(""),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub(_nh, "/image", 1),
     _depth_sub(_nh, "/depth", 1),
     _sync(sync_policy_t(10), _camera_info_sub, _image_sub, _depth_sub),
     _camParam(),
     _useRectifiedImages(false),
     _rightToLeft(),
     _it(_nh),
     _image_pub(_it.advertise("result", 1)),
     _debug_pub(_it.advertise("debug",  1)),
     _pose_pub(_nh.advertise<geometry_msgs::PoseStamped>("pose", 100)),
     _ddr(),
     _marker_detector(),
     _marker_map(),
     _marker_size(0.05),
     _marker_id(26),
     _useSimilarity(false),
     _planarityTolerance(0.001)
{
    _nh.param("marker_size",	    _marker_size,	 _marker_size);
    _nh.param("marker_id",	    _marker_id,		 _marker_id);
    _nh.param("marker_frame",	    _marker_frame,	 _marker_frame);
    _nh.param("camera_frame",	    _camera_frame,	 _camera_frame);
    _nh.param("reference_frame",    _reference_frame,    _reference_frame);
    _nh.param("image_is_rectified", _useRectifiedImages, _useRectifiedImages);

    ROS_INFO_STREAM("Aruco node started with marker size of "
		    << _marker_size << " m and marker id to track: "
		    << _marker_id);
    ROS_INFO_STREAM("Aruco node will publish pose to TF with "
		    << _reference_frame << " as parent and "
		    << _marker_frame << " as child.");

  // Restore marker map if specified.
    std::string	mMapFile;
    _nh.param("marker_map", mMapFile, std::string(""));
    if (mMapFile != "")
	_marker_map.readFromFile(mMapFile);
    else if (_marker_id != 0)
    {
	const auto	half_size = _marker_size/2;
	marker_info_t	mInfo(_marker_id);
	mInfo.push_back({-half_size,  half_size, 0});
	mInfo.push_back({ half_size,  half_size, 0});
	mInfo.push_back({ half_size, -half_size, 0});
	mInfo.push_back({-half_size, -half_size, 0});
	_marker_map.push_back(mInfo);
	_marker_map.mInfoType = marker_map_t::METERS;
    }

  // Set minimum marker size and setup ddynamic_reconfigure service for it.
    double	minMarkerSize = _marker_detector.getParameters().minSize;
    _nh.param("min_marker_size", minMarkerSize, minMarkerSize);

    _ddr.registerVariable<double>("min_marker_size", minMarkerSize,
				  boost::bind(&Detector::set_min_marker_size,
					      this, _1),
				  "Minimum marker size", 0.0, 1.0);
    ROS_INFO_STREAM("Minimum marker size: "
		    << _marker_detector.getParameters().minSize);

  // Set detection mode and setup ddynamic_reconfigure service for it.
    int	detectionMode = _marker_detector.getDetectionMode();
    _nh.param("detection_mode", detectionMode, detectionMode);

    std::map<std::string, int>	map_detectionMode =
    				{
				    {"NORMAL",	   aruco::DM_NORMAL},
				    {"FAST",	   aruco::DM_FAST},
				    {"VIDEO_FAST", aruco::DM_VIDEO_FAST},
				};
    _ddr.registerEnumVariable<int>("detection_mode", detectionMode,
				   boost::bind(&Detector::set_detection_mode,
					       this, _1),
				   "Corner refinement method",
				   map_detectionMode);
    ROS_INFO_STREAM("Detection mode: " << _marker_detector.getDetectionMode());

  // Set dictionary and setup ddynamic_reconfigure service for it.
    int	dictType = aruco::Dictionary::ARUCO;
    _nh.param("dictionary", dictType, dictType);
    set_dictionary(dictType);

    std::map<std::string, int>
	map_dictType =
	{
	    {"ARUCO",		 aruco::Dictionary::ARUCO},
	    {"ARUCO_MIP_25h7",	 aruco::Dictionary::ARUCO_MIP_25h7},
	    {"ARUCO_MIP_16h3",	 aruco::Dictionary::ARUCO_MIP_16h3},
	    {"ARTAG",		 aruco::Dictionary::ARTAG},
	    {"ARTOOLKITPLUS",	 aruco::Dictionary::ARTOOLKITPLUS},
	    {"ARTOOLKITPLUSBCH", aruco::Dictionary::ARTOOLKITPLUSBCH},
	    {"TAG16h5",		 aruco::Dictionary::TAG16h5},
	    {"TAG25h7",		 aruco::Dictionary::TAG25h7},
	    {"TAG25h9",		 aruco::Dictionary::TAG25h9},
	    {"TAG36h11",	 aruco::Dictionary::TAG36h11},
	    {"TAG36h10",	 aruco::Dictionary::TAG36h10},
	    {"CUSTOM",		 aruco::Dictionary::CUSTOM},
	};
    _ddr.registerEnumVariable<int>("dictionary", dictType,
				   boost::bind(&Detector::set_dictionary,
					       this, _1),
				   "Dictionary", map_dictType);
    ROS_INFO_STREAM("Dictionary: " << dictType);

  // Set usage of rigid transformation and setup its dynamic reconfigure service.
    _nh.param("use_similarity", _useSimilarity, _useSimilarity);
    _ddr.registerVariable<bool>(
	"use_similarity", &_useSimilarity,
	"Use similarity transformation to determine marker poses.",
	false, true);

  // Set planarity tolerance and setup its ddynamic_recoconfigure service.
    _nh.param("planarity_tolerance", _planarityTolerance, _planarityTolerance);
    _ddr.registerVariable<double>(
    	"planarity_tolerance", &_planarityTolerance,
    	"Planarity tolerance for extracting marker region(in meters)",
    	0.0005, 0.05);

  // Pulish ddynamic_reconfigure service.
    _ddr.publishServicesTopics();

  // Register callback for marker detection.
    _sync.registerCallback(&Detector::detect_marker_cb, this);
}

void
Detector::run()
{
    ros::spin();
}

void
Detector::set_detection_mode(int mode)
{
    _marker_detector.setDetectionMode(mode,
				      _marker_detector.getParameters().minSize);
}

void
Detector::set_min_marker_size(double size)
{
    _marker_detector.setDetectionMode(_marker_detector.getDetectionMode(),
				      size);
}

void
Detector::set_dictionary(int dict_type)
{
    _marker_detector.setDictionary(dict_type);
}

void
Detector::detect_marker_cb(const camera_info_p& camera_info_msg,
			   const image_p& image_msg, const image_p& depth_msg)
{
    if (_camera_frame.empty())
	_camera_frame = camera_info_msg->header.frame_id;
    if (_reference_frame.empty())
	_reference_frame = _camera_frame;

    try
    {
      // Convert camera_info to aruco camera parameters
	_camParam = rosCameraInfo2ArucoCamParams(*camera_info_msg,
						 _useRectifiedImages);

      // Handle cartesian offset between stereo pairs.
      // See the sensor_msgs/CamaraInfo documentation for details.
	_rightToLeft.setIdentity();
	_rightToLeft.setOrigin(
	    tf::Vector3(-camera_info_msg->P[3]/camera_info_msg->P[0],
			-camera_info_msg->P[7]/camera_info_msg->P[5],
			0.0));

      // Convert sensor_msgs::ImageConstPtr to CvImagePtr.
	auto	image = cv_bridge::toCvCopy(image_msg,
					    sensor_msgs::image_encodings::RGB8)
		      ->image;

      // Detect markers. Results will go into "markers"
	std::vector<aruco::Marker>	markers;
	_marker_detector.detect(image, markers, _camParam, _marker_size, false);

	if (_marker_map.size() > 0)
	{
	    std::vector<std::pair<point3_t, point3_t> >	pairs;

	    for (const auto& marker : markers)
	    {
		const auto	i = _marker_map.getIndexOfMarkerId(marker.id);
		if (i == -1)
		    continue;

		const auto	corners = get_marker_corners(marker,
							     *depth_msg, image);
		const auto&	markerinfo = _marker_map[i];

		for (size_t j = 0; j < corners.size(); ++j)
		    pairs.push_back(std::make_pair(markerinfo[j], corners[j]));

		marker.draw(image, cv::Scalar(0, 0, 255), 2);
	    }

	    publish_transform(pairs.begin(), pairs.end(),
			      depth_msg->header.stamp, _marker_frame);
	}
	else
	{
	  // for each marker, draw info and its boundaries in the image
	    for (const auto& marker : markers)
	    {
		try
		{
		    const auto	corners = get_marker_corners(marker,
							     *depth_msg, image);
		    if (corners.size() < 4)
			throw std::runtime_error("Not all four corners deteced!");

		    std::vector<std::pair<point3_t, point3_t> >	pairs;
		    const auto	half_size = _marker_size/2;
		    pairs.push_back(std::make_pair(
					point3_t(-half_size, -half_size, 0),
					corners[0]));
		    pairs.push_back(std::make_pair(
					point3_t(-half_size,  half_size, 0),
					corners[1]));
		    pairs.push_back(std::make_pair(
					point3_t( half_size,  half_size, 0),
					corners[2]));
		    pairs.push_back(std::make_pair(
					point3_t( half_size, -half_size, 0),
					corners[3]));

		    publish_transform(pairs.begin(), pairs.end(),
				      depth_msg->header.stamp,
				      _marker_frame + '_'
						    + std::to_string(marker.id));
		}
		catch (const std::runtime_error& e)
		{
		    ROS_WARN_STREAM(e.what());
		}

	      // but drawing all the detected markers
		marker.draw(image, cv::Scalar(0, 0, 255), 2);
	    }
	}

	for (auto& marker : markers)
	    aruco::CvDrawingUtils::draw3dAxis(image, marker, _camParam);

	publish_image(image, image_msg->header);
    }
    catch (const std::exception& e)
    {
	ROS_WARN_STREAM(e.what());
    }
}

template <class ITER> void
Detector::publish_transform(ITER begin, ITER end, const ros::Time& stamp,
			    const std::string& marker_frame)
{
    using element_t	= typename std::iterator_traits<ITER>::value_type
				      ::first_type::value_type;
    using matrix33_t	= cv::Matx<element_t, 3, 3>;
    using vector3_t	= cv::Matx<element_t, 3, 1>;

    matrix33_t	R;
    vector3_t	t;
    if (_useSimilarity)
    {
	Similarity<element_t, 3>	similarity;
	const auto	residual = similarity.fit(begin, end);

	ROS_DEBUG_STREAM("Fitted similarity transformation: scale = "
			 << similarity.s() << ", residual = " << residual);

	R = similarity.R();
	t = similarity.t() * (1/similarity.s());
    }
    else
    {
	Rigidity<element_t, 3>	rigidity;
	const auto	residual = rigidity.fit(begin, end);

	ROS_DEBUG_STREAM("Fitted rigid transformation: residual = " << residual);

	R = rigidity.R();
	t = rigidity.t();
    }
    tf::Transform	transform(tf::Matrix3x3(R(0, 0), R(0, 1), R(0, 2),
						R(1, 0), R(1, 1), R(1, 2),
						R(2, 0), R(2, 1), R(2, 2)),
				  tf::Vector3(t(0), t(1), t(2)));

    tf::StampedTransform	cameraToReference;
    cameraToReference.setIdentity();
    if (_reference_frame != _camera_frame)
	get_transform(_reference_frame, _camera_frame, cameraToReference);

    _tfBroadcaster.sendTransform({static_cast<tf::Transform>(cameraToReference) *
				  static_cast<tf::Transform>(_rightToLeft) *
				  transform,
				  stamp, _reference_frame, marker_frame});

    if (_pose_pub.getNumSubscribers() > 0)
    {
	geometry_msgs::PoseStamped	poseMsg;
	tf::poseTFToMsg(transform, poseMsg.pose);
	poseMsg.header.frame_id = _reference_frame;
	poseMsg.header.stamp    = stamp;
	_pose_pub.publish(poseMsg);
    }
}

void
Detector::publish_image(const cv::Mat& image,
			const std_msgs::Header& header) const
{
    if (_image_pub.getNumSubscribers() > 0)
    {
      //show input with augmented information
	cv_bridge::CvImage	out_msg;
	out_msg.header	 = header;
	out_msg.encoding = sensor_msgs::image_encodings::RGB8;
	out_msg.image	 = image;
	_image_pub.publish(out_msg.toImageMsg());
    }

    if (_debug_pub.getNumSubscribers() > 0)
    {
      //show also the internal image
      //resulting from the threshold operation
	cv_bridge::CvImage	debug_msg;
	debug_msg.header   = header;
	debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
	debug_msg.image	   = _marker_detector.getThresholdedImage();
	_debug_pub.publish(debug_msg.toImageMsg());
    }
}

bool
Detector::get_transform(const std::string& refFrame,
			const std::string& childFrame,
			tf::StampedTransform& transform) const
{
    std::string	errMsg;
    if (!_tfListener.waitForTransform(refFrame, childFrame,
				      ros::Time(0), ros::Duration(0.5),
				      ros::Duration(0.01), &errMsg))
    {
	ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
	return false;
    }

    try
    {
	_tfListener.lookupTransform(refFrame, childFrame,
				    ros::Time(0),	//get latest available
				    transform);
    }
    catch (const tf::TransformException& e)
    {
	ROS_ERROR_STREAM("Error in lookupTransform of "
			 << childFrame << " in " << refFrame);
	return false;
    }

    return true;
}

std::vector<Detector::point3_t>
Detector::get_marker_corners(const aruco::Marker& marker,
			     const image_t& depth_msg, cv::Mat& image) const
{
    struct rgb_t	{ uint8_t r, g, b; };
    using element_t	= point3_t::value_type;
    using point_t	= cv::Vec<element_t, 3>;
    using plane_t	= Plane<element_t, 3>;

    std::vector<point3_t>	corners;

    if (marker.size() < 4)
	return corners;

  // Compute initial marker plane.
    std::vector<point3_t>	points;
    for (const auto& corner : marker)
    {
	const auto	point = at<element_t>(depth_msg, corner.x, corner.y);

	if (point(2) != element_t(0))
	    points.push_back(point);
    }
    plane_t	plane(points.cbegin(), points.cend());

  // Compute 2D bounding box of marker.
    const int	u0 = std::floor(std::min({marker[0].x, marker[1].x,
					  marker[2].x, marker[3].x}));
    const int	v0 = std::floor(std::min({marker[0].y, marker[1].y,
					  marker[2].y, marker[3].y}));
    const int	u1 = std::ceil( std::max({marker[0].x, marker[1].x,
					  marker[2].x, marker[3].x}));
    const int	v1 = std::ceil( std::max({marker[0].y, marker[1].y,
					  marker[2].y, marker[3].y}));

  // Select 3D points close to the initial plane within the bounding box.
    points.clear();
    for (auto v = v0; v <= v1; ++v)
	for (auto u = u0; u <= u1; ++u)
	{
	    const auto	point = at<element_t>(depth_msg, u, v);

	    if (point(2) != element_t(0) &&
		plane.distance(point) < _planarityTolerance)
	    {
		points.push_back(point);
		image.at<rgb_t>(v, u).b = 0;
	    }
	}

  // Fit a plane to seleceted inliers.
    plane.fit(points.cbegin(), points.cend());

  // Compute 3D coordinates of marker corners and then publish.
    for (const auto& corner : marker)
	corners.push_back(plane.cross_point(view_vector(corner.x, corner.y)));

    return corners;
}

template <class T> inline cv::Vec<T, 3>
Detector::view_vector(T u, T v) const
{
    cv::Mat_<cv::Point_<T> >	uv(1, 1), xy(1, 1);
    uv(0) = {u, v};
    cv::undistortPoints(uv, xy, _camParam.CameraMatrix, _camParam.Distorsion);

    return {xy(0).x, xy(0).y, T(1)};
}

template <class T> inline cv::Vec<T, 3>
Detector::at(const image_t& depth_msg, int u, int v) const
{
    const auto	xyz = view_vector<T>(u, v);
    const auto	d   = val<T>(depth_msg, u, v);

    return {xyz[0]*d, xyz[1]*d, d};
}

template <class T> cv::Vec<T, 3>
Detector::at(const image_t& depth_msg, T u, T v) const
{
    const int	u0 = std::floor(u);
    const int	v0 = std::floor(v);
    const int	u1 = std::ceil(u);
    const int	v1 = std::ceil(v);
    for (auto vv = v0; vv <= v1; ++vv)
	for (auto uu = u0; uu <= u1; ++uu)
	{
	    const auto	xyz = at<T>(depth_msg, uu, vv);
	    if (xyz[2] != T(0))
		return xyz;
	}

    return {T(0), T(0), T(0)};
}

}	// namespace aist_aruco_ros
