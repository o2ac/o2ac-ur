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
/*!
 *  \file	KLTTracker.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "KLTTracker.h"

namespace aist_visual_tracker
{
/************************************************************************
*  static functions							*
************************************************************************/
//! Get pointer to the leftmost pixel of v-th scanline of the image.
template <class T> inline T*
ptr(sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<T*>(image.data.data() + v*image.step);
}

//! Get const pointer to the leftmost pixel of v-th scanline of the image.
template <class T> inline const T*
ptr(const sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<const T*>(image.data.data() + v*image.step);
}

static std::ostream&
operator <<(std::ostream& out, const KLT_TrackingContextRec& ctx)
{
    return out << "--- Basic parameters ---"
	       << "\nmindist:\t\t"		<< ctx.mindist
	       << "\nwinsize:\t\t"		<< ctx.window_width << 'x'
						<< ctx.window_height
	       << "\nsmoothBeforeSelecting:\t"	<< ctx.smoothBeforeSelecting
	       << "\nwriteInternalImages:\t"	<< ctx.writeInternalImages
	       << "\nlighting_insensitive:\t"	<< ctx.lighting_insensitive
	       << "\n--- Affine parameters ---"
	       << "\naffine_winsize:\t"		<< ctx.affine_window_width << 'x'
						<< ctx.affine_window_height
	       << "\naffineConsistencyCheck:\t"	<< ctx.affineConsistencyCheck;
}

/************************************************************************
*  class KLTTracker							*
************************************************************************/
KLTTracker::KLTTracker(const ros::NodeHandle& nh)
    :_nh(nh),
     _select_srv(_nh.advertiseService("select", &select_cb, this)),
     _it(_nh),
     _image_sub(_it.subscribe("/image", 1, &track_cb, this)),
     _image_pub(_it.advertise("result_image", 1)),
     _ddr(_nh),
     _nfeatures(30),
     _nframes(30),
     _replace(true),
     _ctx(KLTCreateTrackingContext()),
     _featureList(nullptr),
     _featureTable(nullptr),
     _select(true),
     _frame(0),
     _previous_width(0),
     _previous_height(0),
     _marker_size(10),
     _marker_thickness(2)
{
  // Setup ROS parameters for the tracker context only settable at the startup
    _nh.param<int>("min_eigenvalue",
		   _ctx->min_eigenvalue, _ctx->min_eigenvalue);
    _nh.param<float>("min_determinant",
		     _ctx->min_determinant, _ctx->min_determinant);
    _nh.param<float>("min_displacement",
		     _ctx->min_displacement, _ctx->min_displacement);

  // Setup ROS parameters for the tracker context
  // and register callback for dynamic_reconfigure server
    _nh.param<int>("mindist", _ctx->mindist, _ctx->mindist);
    _ddr.registerVariable<int>("mindist", _ctx->mindist,
    			       boost::bind(&KLTTracker::set_ctx_cb<int>, this,
    					   &context_t::mindist,
					   _1),
    			       "The minimum distance between each feature being selected, in pixels",
    			       5, 100);
    int		window_radius;
    _nh.param<int>("window_radius", window_radius, _ctx->window_width/2);
    _ddr.registerVariable<int>("window_radius", window_radius,
			       boost::bind(&KLTTracker::set_window_radius_cb,
					   this,
					   &context_t::window_width,
					   &context_t::window_height,
					   _1),
			       "The radius of the feature window, in pixels",
			       1, 10);
    bool	lighting_insensitive;
    _nh.param<bool>("lighting_insensitive",
		    lighting_insensitive, _ctx->lighting_insensitive);
    _ddr.registerVariable<bool>("lighting_insensitive", lighting_insensitive,
    				boost::bind(&KLTTracker::set_ctx_cb<KLT_BOOL>,
					    this,
					    &context_t::lighting_insensitive,
					    _1),
    				"Whether to normalize the image intensities for gain and bias within the window");
    bool	smooth_before_selecting;
    _nh.param<bool>("lighting_insensitive",
		    smooth_before_selecting, _ctx->smoothBeforeSelecting);
    _ddr.registerVariable<bool>("smooth_before_selecting",
				smooth_before_selecting,
    				boost::bind(&KLTTracker::set_ctx_cb<KLT_BOOL>,
					    this,
					    &context_t::smoothBeforeSelecting,
					    _1),
    				"Whether to smooth image before selecting features");
    int		affine_window_radius;
    _nh.param<int>("affine_window_radius",
		   affine_window_radius, _ctx->affine_window_width/2);
    _ddr.registerVariable<int>("affine_window_radius", affine_window_radius,
			       boost::bind(&KLTTracker::set_window_radius_cb,
					   this,
					   &context_t::affine_window_width,
					   &context_t::affine_window_height,
					   _1),
			       "The width and height of the window used for the affine consistency check",
			       1, 20);
    _nh.param<int>("affine_consistency_check",
		   _ctx->affineConsistencyCheck, _ctx->affineConsistencyCheck);
    const std::map<std::string, int>	enum_check = {{"None",       -1},
						      {"Translation", 0},
						      {"Similarity",  1},
						      {"Affine",      2}};
    _ddr.registerEnumVariable<int>("affine_consistency_check",
				   _ctx->affineConsistencyCheck,
				   boost::bind(&KLTTracker::set_ctx_cb<int>,
					       this,
					       &context_t
					         ::affineConsistencyCheck,
					       _1),
				   "Governs whether to evaluates the consistency of features with affine mapping",
				   enum_check);

  // Setup ROS parameters for the node behavior
  // and register callback for dynamic_reconfigure server
    _nh.param("nfeatures", _nfeatures, _nfeatures);
    _ddr.registerVariable<int>("nfeatures", _nfeatures,
			       boost::bind(&KLTTracker::set_param_cb<int>,
					   this, &KLTTracker::_nfeatures,
					   _1, true),
			       "Number of features tracked",
			       1, 200);
    _nh.param("nframes", _nframes, _nframes);
    _ddr.registerVariable<int>("nframes", _nframes,
			       boost::bind(&KLTTracker::set_param_cb<int>,
					   this, &KLTTracker::_nframes,
					   _1, true),
			       "Number of past frames recoreded",
			       2, 200);
    _nh.param("replace", _replace, _replace);
    _ddr.registerVariable<bool>("replace", _replace,
				boost::bind(&KLTTracker::set_param_cb<bool>,
					    this, &KLTTracker::_replace,
					    _1, false),
				"Replace new features if true");
    _nh.param("marker_size", _marker_size, _marker_size);
    _ddr.registerVariable<int>("marker_size", _marker_size,
			       boost::bind(&KLTTracker::set_param_cb<int>,
					   this, &KLTTracker::_marker_size,
					   _1, false),
			       "Size of Markers for tracked features",
			       1, 80);
    _nh.param("marker_thickness", _marker_thickness, _marker_thickness);
    _ddr.registerVariable<int>("marker_thickness", _marker_thickness,
			       boost::bind(&KLTTracker::set_param_cb<int>,
					   this, &KLTTracker::_marker_thickness,
					   _1, false),
			       "Thickness of markers for tracked features",
			       1, 10);
    _ddr.publishServicesTopics();

  // Set verbosity
    bool	verbose;
    _nh.param("verbose", verbose, false);
    KLTSetVerbosity(verbose);
}

KLTTracker::~KLTTracker()
{
    if (_featureTable)
	KLTFreeFeatureTable(_featureTable);
    if (_featureList)
	KLTFreeFeatureList(_featureList);
    if (_ctx)
	KLTFreeTrackingContext(_ctx);
}

void
KLTTracker::run()
{
    ros::spin();
}

bool
KLTTracker::select_cb(std_srvs::Trigger::Request&  req,
		      std_srvs::Trigger::Response& res)
{
  //KLTPrintTrackingContext(_ctx);
    _select = true;

    res.success = true;
    res.message = "succeeded.";

    ROS_INFO_STREAM("(KLTTracker) select features");

    return true;
}

void
KLTTracker::set_window_radius_cb(int context_t::* width,
				 int context_t::* height, int radius)
{
    _ctx->*width  = 2*radius + 1;
    _ctx->*height = 2*radius + 1;
    KLTUpdateTCBorder(_ctx);
    _select = true;
  //KLTPrintTrackingContext(_ctx);
}

template <class T> void
KLTTracker::set_ctx_cb(T context_t::* field, T value)
{
    _ctx->*field = value;
    _select = true;
  //KLTPrintTrackingContext(_ctx);
}

template <class T> void
KLTTracker::set_param_cb(T KLTTracker::* field, T value, bool select)
{
    this->*field = value;
    _select = select;
}

void
KLTTracker::track_cb(const image_cp& image)
{
    using namespace	sensor_msgs;

  // Convert to grey if the input image is of RGB.
    if (image->encoding == image_encodings::MONO8 ||
	image->encoding == image_encodings::TYPE_8UC1)
    {
	track(*image);	// Track features in the original grey image
    }
    else
    {
	image_t	grey;
	grey.header	  = image->header;
	grey.encoding	  = image_encodings::MONO8;
	grey.height	  = image->height;
	grey.width	  = image->width;
	grey.step	  = grey.width * sizeof(uint8_t);
	grey.is_bigendian = false;
	grey.data.resize(grey.height * grey.step);

	if (image->encoding == image_encodings::RGB8 ||
	    image->encoding == image_encodings::TYPE_8UC3)
	{
	    using	rgb_t = std::array<uint8_t, 3>;

	    for (size_t v = 0; v < image->height; ++v)
	    {
		const auto	p = ptr<rgb_t>(*image, v);
		std::transform(p, p + image->width, ptr<uint8_t>(grey, v),
			       [](const auto& rgb)
			       {
				   return uint8_t(0.299f*rgb[0] +
						  0.587f*rgb[1] +
						  0.114f*rgb[2]);
			       });
	    }
	}
	else if (image->encoding == image_encodings::YUV422)
	{
	    using	yuv422_t = std::array<uint8_t, 2>;	// UYVY format

	    for (size_t v = 0; v < image->height; ++v)
	    {
		const auto	p = ptr<yuv422_t>(*image, v);
		std::transform(p, p + image->width, ptr<uint8_t>(grey, v),
			       [](const auto& yuv422){ return yuv422[1]; });
	    }
	}
	else
	{
	    ROS_ERROR_STREAM("(KLTTracker) Unknown image encoding: "
			     << image->encoding);
	    return;
	}

	track(grey);	// Track features in the converted grey image
    }

  // Draw feature trajectories to a new RGB image and publish.
    cv_bridge::CvImagePtr	cv_img;
    try
    {
	cv_img = cv_bridge::toCvCopy(image, image_encodings::RGB8);
    }
    catch (const cv_bridge::Exception& err)
    {
	ROS_ERROR_STREAM("(KLTTracker) cv_bridge exception: " << err.what());
	return;
    }

    for (size_t j = 0; j < nfeatures(); ++j)
    {
	const auto	feature = (*this)[j];
	if (feature->val >= 0)
	{
	  // 過去に遡る．
	    auto 	fc = feature;
	    bool	stationary = true;
	    for (size_t i = 0; (fc->val == 0) &&	// 最初の点でない？
			       (++i < nframes()); )
	    {
		const auto	fp = (*this)(i, j);	// i時点前のj番目の点
		cv::line(cv_img->image,
			 cv::Point(int(fc->x), int(fc->y)),
			 cv::Point(int(fp->x), int(fp->y)),
			 cv::Scalar(255, 255, 0), _marker_thickness);

		if (std::abs(fc->x - fp->x) > 1.1f ||
		    std::abs(fc->y - fp->y) > 1.1f)
		    stationary = false;

		fc = fp;
	    }

	    const auto	color = (stationary ? cv::Scalar(0, 255, 0)
					    : cv::Scalar(255, 0, 0));
	    cv::drawMarker(cv_img->image,
			   cv::Point(int(feature->x), int(feature->y)), color,
			   cv::MARKER_CROSS, _marker_size, _marker_thickness);
	    cv::putText(cv_img->image, std::to_string(j),
			cv::Point(int(feature->x) + _marker_size,
				  int(feature->y)),
			cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
	}
    }

    _image_pub.publish(cv_img->toImageMsg());
}

void
KLTTracker::track(const image_t& image)
{
  // If image size is changed, refresh tracking context.
    if (image.width  != _previous_width ||
	image.height != _previous_height)
    {
	const auto	ctx = KLTCreateTrackingContext();

	if (_ctx)
	{
	    const auto	pyramid_last	   = ctx->pyramid_last;
	    const auto	pyramid_last_gradx = ctx->pyramid_last_gradx;
	    const auto	pyramid_last_grady = ctx->pyramid_last_grady;
	    *ctx = *_ctx;
	    ctx->pyramid_last	    = pyramid_last;
	    ctx->pyramid_last_gradx = pyramid_last_gradx;
	    ctx->pyramid_last_grady = pyramid_last_grady;
	    KLTFreeTrackingContext(_ctx);
	}

	_ctx = ctx;
	_ctx->sequentialMode = true;	// Always turn on sequential mode
	_ddr.updatePublishedInformation();

	_previous_width  = image.width;
	_previous_height = image.height;
	_select = true;			// Discard features tracked so far
    }

  // Select new features if the tracking context parameters or image sizes
  // have been changed.
    if (_select)
    {
	if (_featureTable)
	    KLTFreeFeatureTable(_featureTable);
	if (_featureList)
	    KLTFreeFeatureList(_featureList);
	_featureList  = KLTCreateFeatureList(_nfeatures);
	_featureTable = KLTCreateFeatureTable(_nframes, _nfeatures);

	_frame = 0;
	selectGoodFeatures(image);
	_select = false;	// Track selected features in subsequent frames
    }

  // Track features.
    trackFeatures(image);
}

KLT_Feature
KLTTracker::operator ()(size_t i, size_t j) const
{
    size_t	frame = _frame;
    while (frame < i)
	frame += nframes();
    return _featureTable->feature[j][frame-i];
}

void
KLTTracker::selectGoodFeatures(const image_t& image)
{
    KLTSelectGoodFeatures(_ctx, const_cast<uint8_t*>(image.data.data()),
			  image.width, image.height, _featureList);
    KLTStoreFeatureList(_featureList, _featureTable, _frame);
}

void
KLTTracker::trackFeatures(const image_t& image)
{
    if (++_frame == _featureTable->nFrames)
	_frame = 0;
    KLTTrackFeatures(_ctx,
		     const_cast<uint8_t*>(image.data.data()),
		     const_cast<uint8_t*>(image.data.data()),
		     image.width, image.height, _featureList);
    if (_replace)
	KLTReplaceLostFeatures(_ctx, const_cast<uint8_t*>(image.data.data()),
			       image.width, image.height, _featureList);
    KLTStoreFeatureList(_featureList, _featureTable, _frame);
}

}	// namespace aist_visual_tracker
