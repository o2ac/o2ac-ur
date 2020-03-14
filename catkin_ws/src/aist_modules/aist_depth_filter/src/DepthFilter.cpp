/*!
 *  \file	DepthFilter.cpp
 *  \author	Toshio UESHIBA
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
#include <cstdlib>	// for getenv()
#include <sys/stat.h>	// for mkdir()
#include <sensor_msgs/image_encodings.h>
#include "tiff.h"
#include "ply.h"
#include "utils.h"
#include "DepthFilter.h"

namespace aist_depth_filter
{
/************************************************************************
*  class DepthFilter							*
************************************************************************/
DepthFilter::DepthFilter(const std::string& name)
    :_nh(name),
     _saveBG_srv(_nh.advertiseService("saveBG", &saveBG_cb, this)),
     _savePly_srv(_nh.advertiseService("savePly", &savePly_cb, this)),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub( _nh, "/image",  1),
     _depth_sub( _nh, "/depth",  1),
     _normal_sub(_nh, "/normal", 1),
     _sync(sync_policy_t(10),
	   _camera_info_sub, _image_sub, _depth_sub, _normal_sub),
     _sync2(sync_policy2_t(10),
     	    _camera_info_sub, _image_sub, _depth_sub),
     _it(_nh),
     _image_pub (_it.advertise("image",  1)),
     _depth_pub( _it.advertise("depth",  1)),
     _normal_pub(_it.advertise("normal", 1)),
     _colored_normal_pub(_it.advertise("colored_normal", 1)),
     _camera_info_pub(_nh.advertise<camera_info_t>("camera_info", 1)),
     _file_info_pub(_nh.advertise<file_info_t>("file_info", 1)),
     _ddr(),
     _camera_info(),
     _image(),
     _depth(nullptr),
     _bg_depth(nullptr),
     _filtered_depth(),
     _normal(),
     _threshBG(0.0),
     _near(0.0),
     _far(FarMax),
     _top(0),
     _bottom(2048),
     _left(0),
     _right(3072),
     _scale(1.0),
     _window_radius(0)
{
    _nh.param("thresh_bg", _threshBG, 0.0);
    _ddr.registerVariable<double>("thresh_bg", &_threshBG,
				  "Threshold value for background removal",
				  0.0, 0.1);
    _nh.param("near", _near, 0.0);
    _ddr.registerVariable<double>("near", &_near,
				  "Nearest depth value", 0.0, 1.0);
    _nh.param("far", _far, 100.0);
    _ddr.registerVariable<double>("far", &_far,
				  "Farest depth value", 0.0, FarMax);
    _nh.param("top", _top, 0);
    _ddr.registerVariable<int>("top",    &_top,	   "Top of ROI",    0, 2048);
    _nh.param("bottom", _bottom, 2048);
    _ddr.registerVariable<int>("bottom", &_bottom, "Bottom of ROI", 0, 2048);
    _nh.param("left", _left, 0);
    _ddr.registerVariable<int>("left",   &_left,   "Left of ROI",   0, 3072);
    _nh.param("right", _right, 3072);
    _ddr.registerVariable<int>("right",  &_right,  "Right of ROI",  0, 3072);
    _nh.param("scale", _scale, 1.0);
    _ddr.registerVariable<double>("scale", &_scale, "Scale depth", 0.5, 1.5);

    bool	subscribe_normal;
    _nh.param("subscribe_normal", subscribe_normal, true);
    if (subscribe_normal)
    {
	_sync.registerCallback(&DepthFilter::filter_with_normal_cb, this);
    }
    else
    {
	_nh.param("window_radius", _window_radius, 0);
	_ddr.registerVariable<int>("window_radius", &_window_radius,
				   "Window radius", 0, 5);
	_sync2.registerCallback(&DepthFilter::filter_without_normal_cb, this);
    }

    _ddr.publishServicesTopics();

}

void
DepthFilter::run()
{
    ros::spin();
}

bool
DepthFilter::saveBG_cb(std_srvs::Trigger::Request&  req,
		       std_srvs::Trigger::Response& res)
{
    try
    {
	if (!_depth)
	    throw std::runtime_error("no original depth image available!");

	saveTiff(*_depth, open_dir() + "/bg.tif");
	_bg_depth = _depth;
	_depth	  = nullptr;

	res.success = true;
	res.message = "succeeded.";
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::saveBG_cb(): " << err.what());

	res.success = false;
	res.message = "failed.";
    }

    ROS_INFO_STREAM("(DepthFilter) save background image: " << res.message);

    return true;
}

bool
DepthFilter::savePly_cb(std_srvs::Trigger::Request&  req,
			std_srvs::Trigger::Response& res)
{
    try
    {
	if (_filtered_depth.data.empty())
	    throw std::runtime_error("no filtered depth image available!");

	const auto	file_path = open_dir() + "/scene.ply";
	savePly(_camera_info, _image, _filtered_depth, _normal, file_path);
	_filtered_depth.data.clear();

	file_info_t	file_info;
	file_info.file_path = file_path;
	file_info.frame     = _camera_info.header.frame_id;
	_file_info_pub.publish(file_info);

	res.success = true;
	res.message = "succeeded.";
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::savePly_cb(): " << err.what());

	res.success = false;
	res.message = "failed.";
    }

    ROS_INFO_STREAM("(DepthFilter) save as OrderedPly: " << res.message);

    return true;
}

void
DepthFilter::filter_with_normal_cb(const camera_info_cp& camera_info,
				   const image_cp& image,
				   const image_cp& depth,
				   const image_cp& normal)
{
    _top    = std::max(0,     std::min(_top,    int(image->height)));
    _bottom = std::max(_top,  std::min(_bottom, int(image->height)));
    _left   = std::max(0,     std::min(_left,   int(image->width)));
    _right  = std::max(_left, std::min(_right,  int(image->width)));

    try
    {
	using	namespace sensor_msgs;

      // Create camera_info according to ROI.
	_camera_info	    = *camera_info;
	_camera_info.height = _bottom - _top;
	_camera_info.width  = _right  - _left;
	_camera_info.K[2]  -= _left;
	_camera_info.K[5]  -= _top;
	_camera_info.P[2]  -= _left;
	_camera_info.P[6]  -= _top;

	_depth = depth;  // Keep pointer to depth for saving background.
	create_subimage(*image,  _image);
	create_subimage(*depth,  _filtered_depth);
	create_subimage(*normal, _normal);
	create_colored_normal(_normal, _colored_normal);

	if (depth->encoding == image_encodings::MONO16 ||
	    depth->encoding == image_encodings::TYPE_16UC1)
	    filter<uint16_t>(_camera_info, _filtered_depth);
	else if (depth->encoding == image_encodings::TYPE_32FC1)
	    filter<float>(_camera_info, _filtered_depth);

	_camera_info_pub.publish(_camera_info);
	_image_pub.publish(_image);
	_depth_pub.publish(_filtered_depth);
	_normal_pub.publish(_normal);
	_colored_normal_pub.publish(_colored_normal);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::filter_with_normal_cb(): "
			 << err.what());
    }
}

void
DepthFilter::filter_without_normal_cb(const camera_info_cp& camera_info,
				      const image_cp& image,
				      const image_cp& depth)
{
    _top    = std::max(0,     std::min(_top,    int(image->height)));
    _bottom = std::max(_top,  std::min(_bottom, int(image->height)));
    _left   = std::max(0,     std::min(_left,   int(image->width)));
    _right  = std::max(_left, std::min(_right,  int(image->width)));

    try
    {
	using	namespace sensor_msgs;

      // Create camera_info according to ROI.
	_camera_info	    = *camera_info;
	_camera_info.height = _bottom - _top;
	_camera_info.width  = _right  - _left;
	_camera_info.K[2]  -= _left;
	_camera_info.K[5]  -= _top;
	_camera_info.P[2]  -= _left;
	_camera_info.P[6]  -= _top;

	_depth = depth;  // Keep pointer to depth for saving background.
	create_subimage(*image, _image);
	create_subimage(*depth, _filtered_depth);

	if (depth->encoding == image_encodings::MONO16 ||
	    depth->encoding == image_encodings::TYPE_16UC1)
	    filter<uint16_t>(_camera_info, _filtered_depth);
	else if (depth->encoding == image_encodings::TYPE_32FC1)
	    filter<float>(_camera_info, _filtered_depth);

	_camera_info_pub.publish(_camera_info);
	_image_pub.publish(_image);
	_depth_pub.publish(_filtered_depth);
	_normal_pub.publish(_normal);
	_colored_normal_pub.publish(_colored_normal);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::filter_without_cb(): " << err.what());
    }
}

template <class T> void
DepthFilter::filter(const camera_info_t& camera_info, image_t& depth)
{
    if (_threshBG > 0)
    {
	try
	{
	    if (!_bg_depth)
		_bg_depth = loadTiff(open_dir() + "/bg.tif");

	    removeBG<T>(depth, *_bg_depth);
	}
	catch (const std::exception& err)
	{
	    _bg_depth = nullptr;
	    _threshBG = 0;
	}
    }
    if (_near > 0.0 || _far < FarMax)
    {
	z_clip<T>(depth);
    }
    if (_window_radius > 0)
    {
	computeNormal<T>(camera_info, depth);
    }
    if (_scale != 1.0)
    {
	scale<T>(depth);
    }
}

template <class T> void
DepthFilter::removeBG(image_t& depth, const image_t& bg_depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	auto	p = ptr<T>(depth, v);
	auto	b = ptr<T>(bg_depth, v + _top) + _left;
	for (const auto q = p + depth.width; p != q; ++p, ++b)
	    if (*b != 0 && std::abs(fval(*p) - fval(*b)) < _threshBG)
		*p = 0;
    }
}

template <class T> void
DepthFilter::z_clip(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	const auto	p = ptr<T>(depth, v);
	std::replace_if(p, p + depth.width,
			[this](const auto& val)
			{ return (fval(val) < _near || fval(val) > _far); },
			0);
    }
}

template <class T> void
DepthFilter::computeNormal(const camera_info_t& camera_info,
			   const image_t& depth)
{
    using		namespace sensor_msgs;

    using value_t		= double;
    using normal_t		= std::array<float, 3>;
    using colored_normal_t	= std::array<uint8_t, 3>;
    using vector3_t		= cv::Matx<value_t, 3, 1>;
    using matrix33_t		= cv::Matx<value_t, 3, 3>;

  // 0: Allocate image for output normals.
    _normal.header		= depth.header;
    _normal.encoding		= image_encodings::TYPE_32FC3;
    _normal.height		= depth.height;
    _normal.width		= depth.width;
    _normal.step		= _normal.width * sizeof(normal_t);
    _normal.is_bigendian	= false;
    _normal.data.resize(_normal.height * _normal.step);
    std::fill(_normal.data.begin(), _normal.data.end(), 0);

  // 1: Allocate image for output colored normals.
    _colored_normal.header	 = depth.header;
    _colored_normal.encoding	 = image_encodings::RGB8;
    _colored_normal.height	 = depth.height;
    _colored_normal.width	 = depth.width;
    _colored_normal.step	 = _colored_normal.width
				 * sizeof(colored_normal_t);
    _colored_normal.is_bigendian = false;
    _colored_normal.data.resize(_colored_normal.height * _colored_normal.step);
    std::fill(_colored_normal.data.begin(), _colored_normal.data.end(), 0);

  // 2: Compute 3D coordinates.
    cv::Mat_<vector3_t>		xyz(depth.height, depth.width);
    depth_to_points<T>(camera_info, depth, xyz.begin());

  // 3: Compute normals.
    const auto			ws1 = 2 * _window_radius;
    cv::Mat_<int>		n(depth.width - ws1, depth.height);
    cv::Mat_<vector3_t>		c(depth.width - ws1, depth.height);
    cv::Mat_<matrix33_t>	M(depth.width - ws1, depth.height);

  // 3.1: Convovle with a box filter in horizontal direction.
    for (int v = 0; v < n.cols; ++v)
    {
	auto	sum_n = 0;
	auto	sum_c = vector3_t::zeros();
	auto	sum_M = matrix33_t::zeros();
	for (int u = 0; u < ws1; ++u)
	{
	    const auto&	head = xyz(v, u);

	    if (head(2) != 0)
	    {
		++sum_n;
		sum_c += head;
		sum_M += head % head;
	    }
	}

	for (int u = 0; u < n.rows; ++u)
	{
	    const auto&	head = xyz(v, u + ws1);

	    if (head(2) != 0)
	    {
		++sum_n;
		sum_c += head;
 		sum_M += head % head;
	    }

	    n(u, v) = sum_n;
	    c(u, v) = sum_c;
	    M(u, v) = sum_M;

	    const auto&	tail = xyz(v, u);

	    if (tail(2) != 0)
	    {
		--sum_n;
		sum_c -= tail;
		sum_M -= tail % tail;
	    }
	}
    }

  // 3.2: Convolve with a box filter in vertical direction.
    for (int u = 0; u < n.rows; ++u)
    {
	auto	sum_n = 0;
	auto	sum_c = vector3_t::zeros();
	auto	sum_M = matrix33_t::zeros();
	for (int v = 0; v < ws1; ++v)
	{
	    sum_n += n(u, v);
	    sum_c += c(u, v);
	    sum_M += M(u, v);
	}

	auto	norm  = ptr<normal_t>(_normal, _window_radius)
		      + _window_radius + u;
	auto	cnorm = ptr<colored_normal_t>(_colored_normal, _window_radius)
		      + _window_radius + u;
	for (int v = ws1; v < n.cols; ++v)
	{
	    sum_n += n(u, v);
	    sum_c += c(u, v);
	    sum_M += M(u, v);

	    if (sum_n > 3)
	    {
		const auto	A = sum_n * sum_M - sum_c % sum_c;
		vector3_t	evalues;
		matrix33_t	evectors;
		cv::eigen(A, evalues, evectors);	// Fit a plane.
		auto		normal   = evectors.row(2).t();
		auto		distance = normal.dot(sum_c) / sum_n;
		if (distance > 0)
		{
		    distance *= -1;
		    normal   *= -1;
		}

		// std::cerr << "evalues = ("
		// 	  << evalues(0) << ", "
		// 	  << evalues(1) << ", "
		// 	  << evalues(2) << ")" << std::endl;
		// std::cerr << "dist = " << dist << ", norm = ("
		// 	  << norm(0) << ", "
		// 	  << norm(1) << ", "
		// 	  << norm(2) << ")" << std::endl;

		*norm  = {normal(0), normal(1), normal(2)};
		*cnorm = {uint8_t(128 + 127*normal(0)),
			  uint8_t(128 + 127*normal(1)),
			  uint8_t(128 + 127*normal(2))};
	    }

	    const auto	vh = v - ws1;
	    sum_n -= n(u, vh);
	    sum_c -= c(u, vh);
	    sum_M -= M(u, vh);

	    norm  += _normal.width;
	    cnorm += _colored_normal.width;
	}
    }
}

template <class T> void
DepthFilter::scale(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	const auto	p = ptr<T>(depth, v);
	std::transform(p, p + depth.width, p,
		       [this](const auto& val){ return _scale * val; });
    }
}

void
DepthFilter::create_subimage(const image_t& image, image_t& subimage) const
{
    using	namespace sensor_msgs;

    const auto	nbytesPerPixel = image_encodings::bitDepth(image.encoding)/8
			       * image_encodings::numChannels(image.encoding);

    subimage.header	  = image.header;
    subimage.height	  = _bottom - _top;
    subimage.width	  = _right  - _left;
    subimage.encoding	  = image.encoding;
    subimage.is_bigendian = image.is_bigendian;
    subimage.step	  = subimage.width*nbytesPerPixel;
    subimage.data.resize(subimage.height * subimage.step);

    auto p = image.data.begin() + _top*image.step + _left*nbytesPerPixel;
    for (auto q = subimage.data.begin(); q != subimage.data.end();
	 q += subimage.step)
    {
	std::copy_n(p, subimage.width*nbytesPerPixel, q);
	p += image.step;
    }
}

void
DepthFilter::create_colored_normal(const image_t& normal,
				   image_t& colored_normal) const
{
    using	namespace sensor_msgs;

    using normal_t		= std::array<float, 3>;
    using colored_normal_t	= std::array<uint8_t, 3>;

    colored_normal.header	= normal.header;
    colored_normal.height	= normal.height;
    colored_normal.width	= normal.width;
    colored_normal.encoding	= image_encodings::RGB8;
    colored_normal.is_bigendian = normal.is_bigendian;
    colored_normal.step		= colored_normal.width
				* sizeof(colored_normal_t);
    colored_normal.data.resize(colored_normal.height * colored_normal.step);

    for (int v = 0; v < normal.height; ++v)
    {
	const auto	p = ptr<normal_t>(normal, v);
	const auto	q = ptr<colored_normal_t>(colored_normal, v);

	std::transform(p, p + normal.width, q,
		       [](const auto& norm)
		       {
			   return colored_normal_t(
				       {uint8_t(128 + 127*norm[0]),
					uint8_t(128 + 127*norm[1]),
				        uint8_t(128 + 127*norm[2])});
		       });
    }
}

std::string
DepthFilter::open_dir()
{
    const auto	home = getenv("HOME");
    if (!home)
	throw std::runtime_error("Environment variable[HOME] is not set.");

    const auto	dir_name = home + std::string("/.ros")
				+ ros::this_node::getNamespace();
    struct stat	buf;
    if (stat(dir_name.c_str(), &buf) && mkdir(dir_name.c_str(), S_IRWXU))
	throw std::runtime_error("Cannot create " + dir_name + ": "
						  + strerror(errno));

    return dir_name;
}

}	// namespace aist_depth_filter
