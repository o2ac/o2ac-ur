/*!
 *  \file	Camera.cpp
 */
#include "Camera.h"
#include <ros/package.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>

namespace aist_phoxi_camera
{
/************************************************************************
*  global functions							*
************************************************************************/
template <class T> static std::ostream&
operator <<(std::ostream& out, const pho::api::PhoXiFeature<T>& feature)
{
    return out << feature.GetValue();
}

/************************************************************************
*  class Camera								*
************************************************************************/
Camera::Camera(const std::string& name)
    :_nh(name),
     _factory(),
     _device(nullptr),
     _frame(nullptr),
     _frame_id("map"),
     _rate(10.0),
     _D({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
     _K({2215.13350577,    0.0        , 1030.47471121 ,
	    0.0       , 2215.13350577 ,  756.735726174,
            0.0       ,    0.0        ,    1.0        }),
     _pointFormat(XYZ),
     _intensityScale(255.0/4095.0),
     _ddr(),
     _get_device_list_server(
	 _nh.advertiseService("get_device_list",   &get_device_list,	this)),
     _is_acquiring_server(
	 _nh.advertiseService("is_acquiring",	   &is_acquiring,	this)),
     _start_acquisition_server(
	 _nh.advertiseService("start_acquisition", &start_acquisition,	this)),
     _stop_acquisition_server(
	 _nh.advertiseService("stop_acquisition",  &stop_acquisition,	this)),
     _trigger_frame_server(
	 _nh.advertiseService("trigger_frame",	   &trigger_frame,	this)),
     _save_frame_server(
	 _nh.advertiseService("save_frame",	   &save_frame,		this)),
     _get_hardware_identification_server(
	 _nh.advertiseService("get_hardware_identification",
			      &get_hardware_identification, this)),
     _get_supported_capturing_modes_server(
	 _nh.advertiseService("get_supported_capturing_modes",
			      &get_supported_capturing_modes, this)),
     _it(_nh),
     _cloud_publisher(	       _nh.advertise<cloud_t>("pointcloud",	1)),
     _normal_map_publisher(    _it.advertise(	      "normal_map",	1)),
     _depth_map_publisher(     _it.advertise(	      "depth_map",	1)),
     _confidence_map_publisher(_it.advertise(	      "confidence_map", 1)),
     _texture_publisher(       _it.advertise(	      "texture",	1)),
     _camera_info_publisher(   _nh.advertise<cinfo_t>("camera_info",	1))
{
    using namespace	pho::api;

  // Search for a device with specified ID.
    std::string	id;
#if ((PHO_SOFTWARE_VERSION_MAJOR >= 1) && (PHO_SOFTWARE_VERSION_MINOR >= 2))
    _nh.param<std::string>("id", id, "InstalledExamples-basic-example");
#else
    _nh.param<std::string>("id", id,
			   "InstalledExamples-PhoXi-example(File3DCamera)");
#endif
    for (size_t pos; (pos = id.find('\"')) != std::string::npos; )
	id.erase(pos, 1);

    if (!_factory.isPhoXiControlRunning())
    {
	ROS_ERROR_STREAM("PhoXiControll is not running.");
	throw std::runtime_error("");
    }

    for (const auto& devinfo : _factory.GetDeviceList())
	if (devinfo.HWIdentification == id)
	{
	    _device = _factory.Create(devinfo.GetTypeHWIdentification());
	    break;
	}
    if (!_device)
    {
	ROS_ERROR_STREAM("Failed to find camera[" << id << "].");
	throw std::runtime_error("");
    }

  // Connect to the device.
    if (!_device->Connect())
    {
	ROS_ERROR_STREAM("Failed to open camera[" << id << "].");
	throw std::runtime_error("");
    }

  // Stop acquisition.
    _device->StopAcquisition();

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") Initializing configuration.");

  // Setup ddynamic_reconfigure services.
  // -- image resolution --
    auto modes = _device->SupportedCapturingModes.GetValue();
    int idx = 0;
    for (; idx < modes.size(); ++idx)
	if (modes[idx] == _device->CapturingMode)
	    break;
    const std::map<std::string, int>	enum_resolution = {{"2064x1544", 0},
							   {"1032x772",  1}};
    _ddr.registerEnumVariable<int>(
	    "resolution", idx,
	    boost::bind(&Camera::set_resolution, this, _1),
	    "Image resolution", enum_resolution);

  // -- trigger mode --
    std::map<std::string, int>	enum_trigger = {{"Freerun",  0},
						{"Software", 1}};
    _ddr.registerEnumVariable<int>(
	    "trigger_mode", _device->TriggerMode.GetValue(),
	    boost::bind(&Camera::set_feature<PhoXiTriggerMode, int>, this,
			&PhoXi::TriggerMode, _1, true, "trigger_mode"),
	    "Trigger mode", enum_trigger);

  // -- timeout --
    std::map<std::string, int>	enum_timeout = {{"ZeroTimeout", 0},
						{"Infinity",   -1},
						{"LastStored", -2},
						{"Default",    -3}};
    _ddr.registerEnumVariable<int>(
	    "timeout",
	    _device->Timeout.GetValue(),
	    boost::bind(&Camera::set_feature<PhoXiTimeout, int>, this,
			&PhoXi::Timeout, _1, false, "timeout"),
	    "Timeout settings", enum_timeout);

  // -- scan/shutter multiplier --
    _ddr.registerVariable<int>(
	    "scan_multiplier",
	    _device->CapturingSettings->ScanMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::ScanMultiplier,
			_1, "scan_multiplier"),
	    "The number of scans taken and merged to sigle output",
	    1, 50);
    _ddr.registerVariable<int>(
	    "shutter_multiplier",
	    _device->CapturingSettings->ShutterMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::ShutterMultiplier,
			_1, "shutter_multiplier"),
	    "The number of repeats of indivisual pattern",
	    1, 20);

  // -- confidence value --
    _ddr.registerVariable<double>(
	    "confidence",
	    _device->ProcessingSettings->Confidence,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, double>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::Confidence,
			_1, "confidence"),
	    "Confidence value",
	    0.0, 100.0);

  // -- enable/disable publish topics --
    _ddr.registerVariable<bool>(
	    "send_point_cloud",
	    _device->OutputSettings->SendPointCloud,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendPointCloud,
			_1, "send_point_cloud"),
	    "Publish point cloud if set.");
    _ddr.registerVariable<bool>(
	    "send_normal_map",
	    _device->OutputSettings->SendNormalMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendNormalMap,
			_1, "send_normal_map"),
	    "Publish normal map if set.");
    _ddr.registerVariable<bool>(
	    "send_depth_map",
	    _device->OutputSettings->SendDepthMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendDepthMap,
			_1, "send_depth_map"),
	    "Publish depth map if set.");
    _ddr.registerVariable<bool>(
	    "send_confidence_map",
	    _device->OutputSettings->SendConfidenceMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendConfidenceMap,
			_1, "send_confidence_map"),
	    "Publish confidence map if set.");
    _ddr.registerVariable<bool>(
	    "send_texture",
	    _device->OutputSettings->SendTexture,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendTexture,
			_1, "send_texture"),
	    "Publish texture if set.");

  // -- point format --
    std::map<std::string, int>	enum_point_format = {{"None",  0},
						     {"RGB",   1},
						     {"Float", 2}};
    _ddr.registerEnumVariable<int>(
	    "point_format", _pointFormat,
	    boost::bind(&Camera::set_member<int>, this,
			boost::ref(_pointFormat), _1, "point_format"),
	    "Format of points in published point cloud", enum_point_format);

  // -- intensity scale --
    _nh.param<double>("intensity_scale", _intensityScale, 255.0/4095.0);
    _ddr.registerVariable<double>(
	    "intensity_scale", _intensityScale,
	    boost::bind(&Camera::set_member<double>, this,
			boost::ref(_intensityScale), _1, "intensity_scale"),
	    "Multiplier for intensity values of published texture",
	    0.05, 5.0);

    _ddr.publishServicesTopics();

  // Read camera intrinsic parameters from device.
    const auto& calib = _device->CalibrationSettings.GetValue();
    const auto& D     = calib.DistortionCoefficients;
    const auto& K     = calib.CameraMatrix;
    std::copy_n(std::begin(D), std::min(D.size(), _D.size()), std::begin(_D));
    std::copy(K[0], K[3], std::begin(_K));

  // Get frame name from parameter server and set it to _frame_id.
    _nh.param<std::string>("frame", _frame_id, "map");

  // Get rate from parameter server and set it to _rate.
    _nh.param<double>("rate", _rate, 10.0);

  // Set trigger_mode according to the parameter.
    int	trigger_mode;
    _nh.param<int>("trigger_mode", trigger_mode, 1);
    set_feature<PhoXiTriggerMode, int>(&PhoXi::TriggerMode, trigger_mode,
				       true, "trigger_mode");

  // Start acquisition.
    _device->ClearBuffer();
    _device->StartAcquisition();

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") aist_phoxi_camera is active.");
}

Camera::~Camera()
{
    if (_device && _device->isConnected())
    {
	_device->StopAcquisition();
	_device->Disconnect();
    }
}

void
Camera::run()
{
    ros::Rate looprate(_rate);

    while (ros::ok())
    {
	tick();
	ros::spinOnce();
	looprate.sleep();
    }
}

void
Camera::tick()
{
    using namespace	pho::api;

    if (_device->TriggerMode == PhoXiTriggerMode::Freerun &&
	_device->isAcquiring())
    {
	std::lock_guard<std::mutex>	lock(_mutex);

	if ((_frame = _device->GetFrame(PhoXiTimeout::ZeroTimeout)) &&
	    _frame->Successful)
	    publish_frame();
    }
}

double
Camera::rate() const
{
    return _rate;
}

void
Camera::set_resolution(int idx)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    const auto modes = _device->SupportedCapturingModes.GetValue();
    if (idx < modes.size())
	_device->CapturingMode = modes[idx];
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") set resolution to "
		    << _device->CapturingMode.GetValue().Resolution.Width
		    << 'x'
		    << _device->CapturingMode.GetValue().Resolution.Height);

    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();
}

template <class F, class T> void
Camera::set_feature(pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
		    T value, bool pause, const std::string& name)
{
    const auto acq = _device->isAcquiring();
    if (pause && acq)
	_device->StopAcquisition();

    (_device.operator ->()->*feature).SetValue(value);
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") set " << name << " to "
		    << (_device.operator ->()->*feature).GetValue());

    if (pause)
    {
	_device->ClearBuffer();
	if (acq)
	    _device->StartAcquisition();
    }
}

template <class F, class T> void
Camera::set_field(pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
		  T F::* field, T value, const std::string& name)
{
    auto	val = (_device.operator ->()->*feature).GetValue();
    val.*field = value;
    (_device.operator ->()->*feature).SetValue(val);
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") set " << name << " to "
		    << (_device.operator ->()->*feature).GetValue().*field);
}

template <class T> void
Camera::set_member(T& member, T value, const std::string& name)
{
    member = value;
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") set " << name << " to " << member);
}

bool
Camera::get_device_list(GetStringList::Request&  req,
			GetStringList::Response& res)
{
    _factory.StartConsoleOutput("Admin-On");

    const auto	devinfos = _factory.GetDeviceList();

    res.len = devinfos.size();
    for (const auto& devinfo : devinfos)
	res.out.push_back(devinfo.HWIdentification);

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") get_device_list: succeded.");

    return true;
}

bool
Camera::is_acquiring(std_srvs::Trigger::Request&  req,
		     std_srvs::Trigger::Response& res)
{
    res.success = _device->isAcquiring();
    res.message = (res.success ? "yes" : "no");

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") is_acquiring: "
		    << res.message);

    return true;
}

bool
Camera::start_acquisition(std_srvs::Trigger::Request&  req,
			  std_srvs::Trigger::Response& res)
{
    if (_device->isAcquiring())
    {
	res.success = true;
	res.message = "already in aquisition.";
    }
    else
    {
      // Flush buffer to avoid publishing data during past acquisition.
	_device->ClearBuffer();			// MUST!!!
	res.success = _device->StartAcquisition();
	res.message = (res.success ? "scceeded." : "failed.");
    }

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") start_acquisition: "
		    << res.message);

    ros::Duration(1.0).sleep();
    return true;
}

bool
Camera::stop_acquisition(std_srvs::Trigger::Request&  req,
			 std_srvs::Trigger::Response& res)
{
    if (_device->isAcquiring())
    {
	res.success = _device->StopAcquisition();
	res.message = (res.success ? "succeeded." : "failed.");
    }
    else
    {
	res.success = true;
	res.message = "already not in aquisition.";
    }

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") stop_acquisition: "
		    << res.message);

    return true;
}

bool
Camera::trigger_frame(std_srvs::Trigger::Request&  req,
		      std_srvs::Trigger::Response& res)
{
    using namespace	pho::api;

    const auto	frameId = _device->TriggerFrame(true, true);

    switch (frameId)
    {
      case -1:
	res.success = false;
	res.message = "failed. [TriggerFrame not accepted]";
	break;
      case -2:
	res.success = false;
	res.message = "failed. [device is not running]";
	break;
      case -3:
	res.success = false;
	res.message = "failed. [communication error]";
	break;
      case -4:
	res.success = false;
	res.message = "failed. [WaitForGrabbingEnd is not supported]";
	break;
      default:
	if (!(_frame = _device->GetSpecificFrame(frameId,
						 PhoXiTimeout::Infinity)))
	{
	    res.success = false;
	    res.message = "failed. [not found frame #"
			+ std::to_string(frameId) + ']';
	    break;
	}

	if (_frame->Info.FrameIndex != frameId)
	{
	    res.success = false;
	    res.message = "failed. [triggered frame(#"
			+ std::to_string(frameId)
			+ ") is not captured frame(#"
			+ std::to_string(_frame->Info.FrameIndex)
			+ ")]";
	    break;
	}

	publish_frame();	// publish
	res.success = true;
	res.message = "succeeded. [frame #" + std::to_string(frameId) + ']';
	break;
    }

    if (res.success)
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") get_frame: "
			<< res.message);
    else
	ROS_ERROR_STREAM('('
			 << _device->HardwareIdentification.GetValue()
			 << ") get_frame: "
			 << res.message);

    return true;
}

bool
Camera::save_frame(SetString::Request& req, SetString::Response& res)
{
    std::lock_guard<std::mutex>	lock(_mutex);

    if (_frame == nullptr || !_frame->Successful)
    {
	res.success = false;
	ROS_ERROR_STREAM('('
			 << _device->HardwareIdentification.GetValue()
			 << ") save_frame: failed. [no frame data]");
    }
    else if (!_frame->SaveAsPly(req.in + ".ply"))
    {
	res.success = false;
	ROS_ERROR_STREAM('('
			 << _device->HardwareIdentification.GetValue()
			 << ") save_frame: failed to save PLY to "
			 << req.in + ".ply");
    }
    else
    {
	res.success = true;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") save_frame: succeeded to save PLY to "
			<< req.in + ".ply");
    }

    return true;
}

bool
Camera::get_supported_capturing_modes(
		GetSupportedCapturingModes::Request&  req,
	        GetSupportedCapturingModes::Response& res)
{
    const auto	modes = _device->SupportedCapturingModes.GetValue();
    for (const auto& mode : modes)
    {
	aist_phoxi_camera::PhoXiSize	size;
	size.Width  = mode.Resolution.Width;
	size.Height = mode.Resolution.Height;
	res.supported_capturing_modes.push_back(size);
    }

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") get_supported_capturing_moddes: succeeded.");

    return true;
}

bool
Camera::get_hardware_identification(GetString::Request&  req,
				    GetString::Response& res)
{
    res.out = _device->HardwareIdentification;

    return true;
}

void
Camera::publish_frame() const
{
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue() << ") "
		    << "PointCloud: "
		    << _frame->PointCloud.Size.Width << 'x'
		    << _frame->PointCloud.Size.Height
		    << " [frame #" << _frame->Info.FrameIndex << ']');

  // Common setting.
    const auto	now = ros::Time::now();

  // Publish point cloud.
    constexpr float	distanceScale = 0.001;	// milimeters -> meters
    publish_cloud(now, distanceScale);

  // Publish normal_map, depth_map, confidence_map and texture.
    publish_image(_frame->NormalMap, _normal_map_publisher, now,
		  sensor_msgs::image_encodings::TYPE_32FC3, 1);
    publish_image(_frame->DepthMap, _depth_map_publisher, now,
		  sensor_msgs::image_encodings::TYPE_32FC1, distanceScale);
    publish_image(_frame->ConfidenceMap, _confidence_map_publisher, now,
		  sensor_msgs::image_encodings::TYPE_32FC1, 1);
    publish_image(_frame->Texture, _texture_publisher, now,
		  sensor_msgs::image_encodings::MONO8, _intensityScale);

  // publish camera_info
    publish_camera_info(now);
}

void
Camera::publish_cloud(const ros::Time& stamp, float distanceScale) const
{
    using namespace	sensor_msgs;

    const auto&	phoxi_cloud = _frame->PointCloud;
    if (phoxi_cloud.Empty())
	return;

  // Convert pho::api::PointCloud32f to sensor_msgs::PointCloud2
    cloud_p	cloud(new cloud_t);
    cloud->is_bigendian = false;
    cloud->is_dense	= false;

    PointCloud2Modifier	modifier(*cloud);
#if 1
    if (_pointFormat == XYZ)
	modifier.setPointCloud2Fields(3,
				      "x", 1, PointField::FLOAT32,
				      "y", 1, PointField::FLOAT32,
				      "z", 1, PointField::FLOAT32);
    else
	modifier.setPointCloud2Fields(4,
				      "x",   1, PointField::FLOAT32,
				      "y",   1, PointField::FLOAT32,
				      "z",   1, PointField::FLOAT32,
				      "rgb", 1, PointField::FLOAT32);
#else
    if (_pointFormat == XYZ)
	modifier.setPointCloud2FieldsByString(1, "xyz");
    else
	modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
#endif
    modifier.resize(phoxi_cloud.Size.Height * phoxi_cloud.Size.Width);

    cloud->header.stamp	   = stamp;
    cloud->header.frame_id = _frame_id;
    cloud->height	   = phoxi_cloud.Size.Height;
    cloud->width	   = phoxi_cloud.Size.Width;
    cloud->row_step	   = cloud->width * cloud->point_step;

    const auto&	phoxi_texture = _frame->Texture;

    for (int v = 0; v < cloud->height; ++v)
    {
	PointCloud2Iterator<float>	xyz(*cloud, "x");
	xyz += cloud->width * v;

	for (int u = 0; u < cloud->width; ++u)
	{
	    const auto&	p = phoxi_cloud.At(v, u);

	    if (float(p.z) == 0.0f)
	    {
		xyz[0] = xyz[1] = xyz[2]
		       = std::numeric_limits<float>::quiet_NaN();
	    }
	    else
	    {
		xyz[0] = p.x * distanceScale;
		xyz[1] = p.y * distanceScale;
		xyz[2] = p.z * distanceScale;
	    }

	    ++xyz;
	}

	if (_pointFormat == XYZRGB)
	{
	    PointCloud2Iterator<uint8_t> rgb(*cloud, "rgb");
	    rgb += cloud->width * v;

	    for (int u = 0; u < cloud->width; ++u)
	    {
		const auto	val = phoxi_texture.At(v, u) * _intensityScale;

		rgb[0] = rgb[1] = rgb[2] = (val > 255 ? 255 : val);
		++rgb;
	    }
	}
	else if (_pointFormat == XYZI)
	{
	    PointCloud2Iterator<float>	rgb(*cloud, "rgb");
	    rgb += cloud->width * v;

	    for (int u = 0; u < cloud->width; ++u)
	    {
		*rgb = phoxi_texture.At(v, u) * _intensityScale;
		++rgb;
	    }
	}
    }

    _cloud_publisher.publish(cloud);
}

template <class T> void
Camera::publish_image(const pho::api::Mat2D<T>& phoxi_image,
		      const image_transport::Publisher& publisher,
		      const ros::Time& stamp,
		      const std::string& encoding,
		      typename T::ElementChannelType scale) const
{
    using namespace	sensor_msgs;
    using		element_ptr = const typename T::ElementChannelType*;

    if (phoxi_image.Empty())
	return;

    image_p	image(new image_t);
    image->header.stamp    = stamp;
    image->header.frame_id = _frame_id;
    image->encoding	   = encoding;
    image->is_bigendian    = 0;
    image->height	   = phoxi_image.Size.Height;
    image->width	   = phoxi_image.Size.Width;
    image->step		   = image->width
			   *  image_encodings::numChannels(image->encoding)
			   * (image_encodings::bitDepth(image->encoding)/8);
    image->data.resize(image->step * image->height);

    const auto	p = reinterpret_cast<element_ptr>(phoxi_image[0]);
    const auto	q = reinterpret_cast<element_ptr>(
			phoxi_image[phoxi_image.Size.Height]);

    if (image->encoding == image_encodings::MONO8)
	std::transform(p, q,
		       reinterpret_cast<uint8_t*>(image->data.data()),
		       [scale](const auto& x)->uint8_t
		       { auto y = scale * x; return y > 255 ? 255 : y; });
    else if (image->encoding == image_encodings::MONO16)
	std::transform(p, q,
		       reinterpret_cast<uint16_t*>(image->data.data()),
		       [scale](const auto& x)->uint16_t
		       { return scale * x; });
    else if (image->encoding.substr(0, 4) == "32FC")
	std::transform(p, q,
		       reinterpret_cast<float*>(image->data.data()),
		       [scale](const auto& x)->float
		       { return scale * x; });
    else if (image->encoding.substr(0, 4) == "64FC")
	std::transform(p, q,
		       reinterpret_cast<double*>(image->data.data()),
		       [scale](const auto& x)->double
		       { return scale * x; });
    else
    {
	ROS_ERROR_STREAM("Unsupported image type!");
	throw std::logic_error("");
    }

    publisher.publish(image);
}

void
Camera::publish_camera_info(const ros::Time& stamp) const
{
    cinfo_t	cinfo;

  // Set header.
    cinfo.header.stamp    = stamp;
    cinfo.header.frame_id = _frame_id;

  // Set height and width.
    const auto	mode = _device->CapturingMode.GetValue();
    cinfo.height = mode.Resolution.Height;
    cinfo.width  = mode.Resolution.Width;

  // Set distortion and intrinsic parameters.
    cinfo.distortion_model = "plumb_bob";
    cinfo.D.resize(_D.size());
    std::copy(std::begin(_D), std::end(_D), std::begin(cinfo.D));
    std::copy(std::begin(_K), std::end(_K), std::begin(cinfo.K));

  // Set cinfo.R to be an identity matrix.
    std::fill(std::begin(cinfo.R), std::end(cinfo.R), 0.0);
    cinfo.R[0] = cinfo.R[4] = cinfo.R[8] = 1.0;

  // Set 3x4 camera matrix.
    for (int i = 0; i < 3; ++i)
	for (int j = 0; j < 3; ++j)
	    cinfo.P[4*i + j] = cinfo.K[3*i + j];
    cinfo.P[3] = cinfo.P[7] = cinfo.P[11] = 0.0;

  // No binning
    cinfo.binning_x = cinfo.binning_y = 0;

  // ROI is same as entire image.
    cinfo.roi.width = cinfo.roi.height = 0;

    _camera_info_publisher.publish(cinfo);
}

}	// namespace aist_phoxi_camera
