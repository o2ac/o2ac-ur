/*!
 *  \file	Camera.h
 */
#include <string>
#include <array>
#include <mutex>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include <PhoXi.h>

#include <aist_phoxi_camera/SetString.h>
#include <aist_phoxi_camera/GetString.h>
#include <aist_phoxi_camera/GetStringList.h>
#include <aist_phoxi_camera/GetSupportedCapturingModes.h>

namespace aist_phoxi_camera
{
/************************************************************************
*   class Camera                                                        *
************************************************************************/
class Camera
{
  private:
    using cloud_t = sensor_msgs::PointCloud2;
    using cloud_p = sensor_msgs::PointCloud2Ptr;
    using image_t = sensor_msgs::Image;
    using image_p = sensor_msgs::ImagePtr;
    using cinfo_t = sensor_msgs::CameraInfo;

    enum
    {
	XYZ = 0, XYZRGB = 1, XYZI = 2
    };

  public:
		Camera(const std::string& name)				;
		~Camera()						;

    void	run()							;
    void	tick()							;
    double	rate()						const	;

  private:
    void	set_resolution(int idx)					;
    template <class F, class T>
    void	set_feature(pho::api::PhoXiFeature<F> pho::api::PhoXi::*
			      feature,
			    T value,
			    bool reset, const std::string& name)	;
    template <class F, class T>
    void	set_field(pho::api::PhoXiFeature<F> pho::api::PhoXi::*
			      feature,
			  T F::* member,
			  T value, const std::string& name)		;
    template <class T>
    void	set_member(T& member, T value, const std::string& name)	;
    bool	get_device_list(GetStringList::Request&  req,
				GetStringList::Response& res)		;
    bool	is_acquiring(std_srvs::Trigger::Request&  req,
			     std_srvs::Trigger::Response& res)		;
    bool	start_acquisition(std_srvs::Trigger::Request&  req,
				  std_srvs::Trigger::Response& res)	;
    bool	stop_acquisition(std_srvs::Trigger::Request&  req,
				 std_srvs::Trigger::Response& res)	;
    bool	trigger_frame(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    bool	save_frame(SetString::Request&  req,
			   SetString::Response& res)			;
    bool	get_supported_capturing_modes(
			GetSupportedCapturingModes::Request&  req,
			GetSupportedCapturingModes::Response& res)	;
    bool	get_hardware_identification(GetString::Request&  req,
					    GetString::Response& res)	;
    template <class T>
    static bool	save_image(const std::string& filename,
			   const pho::api::Mat2D<T>& phoxi_image,
			   float scale)					;
    void	publish_frame()					const	;
    void	publish_cloud(const ros::Time& stamp,
			      float distanceScale)		const	;
    template <class T>
    void	publish_image(const pho::api::Mat2D<T>& phoxi_image,
			      const image_transport::Publisher& publisher,
			      const ros::Time& stamp,
			      const std::string& encoding,
			      typename T::ElementChannelType scale)
								const	;
    void	publish_camera_info(const ros::Time& stamp)	const	;

  private:
    ros::NodeHandle			_nh;

    mutable std::mutex			_mutex;
    const pho::api::PhoXiFactory	_factory;
    pho::api::PPhoXi			_device;
    pho::api::PFrame			_frame;
    std::string				_frame_id;	// frame id used by tf
    double				_rate;		// frequency
    std::array<double, 8>		_D;		// distortion param.
    std::array<double, 9>		_K;		// intrinsic param.
    int					_pointFormat;
    double				_intensityScale;

    ddynamic_reconfigure::DDynamicReconfigure	_ddr;

    const ros::ServiceServer		_get_device_list_server;
    const ros::ServiceServer		_is_acquiring_server;
    const ros::ServiceServer		_start_acquisition_server;
    const ros::ServiceServer		_stop_acquisition_server;
    const ros::ServiceServer		_trigger_frame_server;
    const ros::ServiceServer		_save_frame_server;
    const ros::ServiceServer		_get_hardware_identification_server;
    const ros::ServiceServer		_get_supported_capturing_modes_server;

    image_transport::ImageTransport	_it;
    const ros::Publisher		_cloud_publisher;
    const image_transport::Publisher	_normal_map_publisher;
    const image_transport::Publisher	_depth_map_publisher;
    const image_transport::Publisher	_confidence_map_publisher;
    const image_transport::Publisher	_texture_publisher;
    const ros::Publisher		_camera_info_publisher;
};

}	// namespace aist_phoxi_camera
