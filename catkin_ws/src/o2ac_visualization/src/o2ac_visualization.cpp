#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <iostream>
#include <nodelet/nodelet.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <stdio.h>
#include <string.h>

using ImageCallback = boost::function<void(const sensor_msgs::ImageConstPtr &)>;
using CamInfoCallback =
    boost::function<void(const sensor_msgs::CameraInfoConstPtr &)>;
using StringCallback =
    boost::function<void(const std_msgs::String::ConstPtr &)>;
using KittingSetIdCallback =
    boost::function<void(const std_msgs::Int32::ConstPtr &)>;
using SuctionSuccessCallback =
    boost::function<void(const std_msgs::Bool::ConstPtr &)>;
using RunModeCallback = boost::function<void(const std_msgs::Bool::ConstPtr &)>;
using RosParam = XmlRpc::XmlRpcValue;

const int font_face_ = cv::FONT_HERSHEY_DUPLEX;
const double font_scale_ = 0.5;
const int font_thick_ = 1;
const int font_ltype_ = 4; // CV_AA;

void check_window() {
  if (!getWindowProperty("Monitor", cv::WND_PROP_AUTOSIZE) >= 0) {
    cv::namedWindow("Monitor", cv::WINDOW_NORMAL);
    cv::moveWindow("Monitor", 128, 128);
  }
}

void clear_buffer_rect(cv::Rect rect, const cv::Mat &monitor_) {
  cv::rectangle(monitor_, cv::Point(rect.x, rect.y),
                cv::Point(rect.x + rect.width, rect.y + rect.height),
                cv::Scalar(0, 0, 0), cv::FILLED);
  cv::rectangle(monitor_, cv::Point(rect.x, rect.y),
                cv::Point(rect.x + rect.width, rect.y + rect.height),
                cv::Scalar(255, 255, 255), 1);
}

void putText(const cv::Mat &monitor_, int x, int y, const std::string &text) {
  cv::Point point(x, y);
  cv::putText(monitor_, text, point, font_face_, 2 * font_scale_,
              cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
}

long now() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// Adopted from team-NAIST-Panasonic ARC repository
// https://github.com/warehouse-picking-automation-challenges/team_naist_panasonic
cv_bridge::CvImagePtr convert2OpenCV(const sensor_msgs::Image &img,
                                     const std::string type_str) {
  auto type_val = sensor_msgs::image_encodings::BGR8;

  if ((type_str == "depth") || (type_str == "DEPTH")) {
    type_val = sensor_msgs::image_encodings::TYPE_32FC1;
    // ROS_INFO("Image encoding: %s", type_val.c_str());
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, type_val);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Image converter, cv_bridge exception: %s", e.what());
  }

  return cv_ptr;
}

//  boost::function<void(const sensor_msgs::ImageConstPtr&)>
ImageCallback getCallbackForImage(cv::Rect rect, XmlRpc::XmlRpcValue &params,
                                  cv::Mat &monitor_) {
  // Values captured by closure
  double vmin = 0;
  double vmax = 255;
  auto img_type_str = sensor_msgs::image_encodings::RGB8;

  if (params.hasMember("vrange") == true) {
    int tmp1 = params["vrange"][0];
    int tmp2 = params["vrange"][1];
    vmin = tmp1;
    vmax = tmp2;
  }

  if (params.hasMember("encoding") == true) {
    img_type_str = std::string(params["encoding"]);
  }

  // This function returns callback function
  return [=](const sensor_msgs::ImageConstPtr &msg) {
    // Show window at the first time
    if (!getWindowProperty("Monitor", cv::WND_PROP_AUTOSIZE) >= 0) {
      cv::namedWindow("Monitor", cv::WINDOW_NORMAL);
      cv::moveWindow("Monitor", 128, 128);
    }

    // Convert sent message into cv::Mat
    cv_bridge::CvImagePtr cv_ptr = convert2OpenCV(*msg, img_type_str);
    cv::Mat img_org =
        cv_ptr->image; // convert2OpenCV(*msg, img_type_str) -> image;
    cv::Mat img;

    // For float image type
    auto encoding = cv_ptr->encoding;
    if (encoding == "32FC1") // assume depth map
    {
      img = cv::Mat(img_org.size(), CV_8UC3);

      for (int i = 0; i < img_org.rows; i++) {
        for (int j = 0; j < img_org.cols; j++) {
          double tmp = img_org.at<float>(i, j);
          tmp = (tmp - vmin) / (vmax - vmin) * 255;
          if (tmp > 255) {
            tmp = 255;
          }
          if (tmp < 0) {
            tmp = 0;
          }
          img.at<cv::Vec3b>(i, j)[0] = (uint8_t)tmp;
          img.at<cv::Vec3b>(i, j)[1] = (uint8_t)tmp;
          img.at<cv::Vec3b>(i, j)[2] = (uint8_t)tmp;
        }
      }
    } else {
      img = img_org;
    }

    // Resize the image and put it to the buffer
    cv::Size size = cv::Size(rect.width, rect.height);
    cv::Mat resized;
    cv::resize(img, resized, size, 0, 0, cv::INTER_CUBIC);

    // Color scaling for BGR8
    if (encoding == "BGR8" || encoding == "RGB8") {
      for (int i = 0; i < resized.rows; i++) {
        for (int j = 0; j < resized.cols; j++) {
          for (int idx = 0; idx < 3; idx++) {
            double tmp = resized.at<cv::Vec3b>(i, j)[idx];
            tmp = (tmp - vmin) / (vmax - vmin) * 255;
            if (tmp > 255) {
              tmp = 255;
            }
            if (tmp < 0) {
              tmp = 0;
            }
            resized.at<cv::Vec3b>(i, j)[idx] = (uint8_t)tmp;
          }
        }
      }
    }

    resized.copyTo(monitor_(rect));

    // Copy the image buffer in the window
    cv::imshow("Monitor", monitor_);
    cv::waitKey(3);
  };
}

// boost::function<void(const sensor_msgs::CameraInfoConstPtr&)>
CamInfoCallback getCallbackForCameraInfo(cv::Rect rect,
                                         XmlRpc::XmlRpcValue &params,
                                         cv::Mat &monitor_) {
  // Values captured by closure
  int offset_x = params["offset"][0];
  int offset_y = params["offset"][1];

  // This function returns callback function
  return [=](const sensor_msgs::CameraInfoConstPtr &msg) {
    // Show window at the first time
    check_window();

    clear_buffer_rect(rect, monitor_);

    std::string text1 = "Width : " + std::to_string(msg->width);
    putText(monitor_, rect.x + offset_x, rect.y + offset_y, text1);
    std::string text2 = "Height: " + std::to_string(msg->height);
    putText(monitor_, rect.x + offset_x, rect.y + offset_y + 32, text2);
  };
}

StringCallback getCallbackForString(cv::Rect rect, XmlRpc::XmlRpcValue &params,
                                    cv::Mat &monitor_, long &start_time) {
  // Values captured by closure
  int offset_x = params["offset"][0];
  int offset_y = params["offset"][1];
  int n_history = params["n_history"];
  printf("%d", n_history);
  std::deque<std::string> messages;
  for (int i = 0; i < n_history; i++) {
    messages.push_back("");
  }
  std::vector<int> ivec;

  // This function returns callback function
  return [=, &start_time](const std_msgs::String::ConstPtr &msg) mutable {
    check_window();
    clear_buffer_rect(rect, monitor_);

    long elapsed_time = now() - start_time;
    char buf[255];
    sprintf(buf, "%6.1lfs: ", (float)elapsed_time / 1000);
    std::string text = buf + std::string(msg->data.c_str());
    messages.push_back(text);
    messages.pop_front();

    for (int i = 0; i < n_history; i++) {
      putText(monitor_, rect.x + offset_x, rect.y + offset_y + 32 * i,
              messages[i]);
    }

    // Copy the image buffer in the window
    cv::imshow("Monitor", monitor_);
    cv::waitKey(3);
  };
}

KittingSetIdCallback getCallbackForKittingSetId(cv::Rect rect,
                                                XmlRpc::XmlRpcValue &params,
                                                cv::Mat &monitor_,
                                                ros::NodeHandle &nh) {
  // Values captured by closure
  int offset_x = params["offset"][0];
  int offset_y = params["offset"][1];

  // This function returns callback function
  return [=](const std_msgs::Int32::ConstPtr &msg) {
    check_window();
    clear_buffer_rect(rect, monitor_);

    RosParam parts_set;
    nh.getParam("/set_" + std::to_string(msg->data), parts_set);

    int i = 0;
    char buf[255];
    sprintf(buf, "Set %d", msg->data);
    putText(monitor_, rect.x + offset_x, rect.y + offset_y + 32 * i, buf);
    i += 2;

    for (auto it = parts_set.begin(); it != parts_set.end(); it++) {
      // auto a = parts_set[i];
      putText(monitor_, rect.x + offset_x, rect.y + offset_y + 32 * i,
              it->first);
      i++;
    }
  };
}

void updateSuctionSuccess(cv::Rect rect, cv::Mat &monitor_,
                          std::vector<std::string> &suction_success_str,
                          int offset_x, int offset_y) {
  check_window();
  clear_buffer_rect(rect, monitor_);

  int i = 0;
  for (auto it = suction_success_str.begin(); it != suction_success_str.end();
       it++) {
    putText(monitor_, rect.x + offset_x, rect.y + offset_y + 32 * i, *it);
    i++;
  }

  // Copy the image buffer in the window
  cv::imshow("Monitor", monitor_);
  cv::waitKey(3);
}

SuctionSuccessCallback
getCallbackForSuctionSuccess(cv::Rect rect, XmlRpc::XmlRpcValue &params,
                             cv::Mat &monitor_,
                             std::vector<std::string> &suction_success_str) {
  // Values captured by closure
  int offset_x = params["offset"][0];
  int offset_y = params["offset"][1];
  int index = params["index"];
  std::string str = params["str"];

  suction_success_str.push_back(std::string("---"));

  // This function returns callback function
  return
      [=, &suction_success_str](const std_msgs::Bool::ConstPtr &msg) mutable {
        check_window();
        char buf[255];
        sprintf(buf, "%s: %s", str.c_str(), msg->data ? "True" : "False");
        suction_success_str[index] = std::string(buf);
        if (index == 0) {
          updateSuctionSuccess(rect, monitor_, suction_success_str, offset_x,
                               offset_y);
        }
      };
}

RunModeCallback getCallbackForRunMode(cv::Rect rect,
                                      XmlRpc::XmlRpcValue &params,
                                      cv::Mat &monitor_) {
  // Values captured by closure
  int offset_x = params["offset"][0];
  int offset_y = params["offset"][1];
  std::string str = params["str"];
  ROS_INFO("%s", str.c_str());
  // int r = params["color"][0];
  // int g = params["color"][1];
  // int b = params["color"][2];

  // This function returns callback function
  return [=](const std_msgs::Bool::ConstPtr &msg) {
    // ROS_INFO("Debug monitor timer reset");
    if (msg->data) {
      check_window();
      clear_buffer_rect(rect, monitor_);
      putText(monitor_, rect.x + offset_x, rect.y + offset_y, str);
      cv::imshow("Monitor", monitor_);
      cv::waitKey(3);
    }
  };
}

void timerCallback(const ros::TimerEvent &) {
  check_window();
  cv::waitKey(3);
}

namespace o2ac_visualization {
class Monitor {
public:
  Monitor(ros::NodeHandle &nh) {
    // Set global parameters
    XmlRpc::XmlRpcValue global;
    nh.getParam("o2ac_visualization/global", global);
    setGlobalParams(global);

    // Initialize shared objects in this class
    monitor_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Start time
    start_time = now();

    // Set callback functions for image topics
    XmlRpc::XmlRpcValue image_topics;
    nh.getParam("o2ac_visualization/image_topics", image_topics);
    setImageCallbacks(nh, image_topics);

    // Set callback functions for CameraInfo topics
    XmlRpc::XmlRpcValue caminfo_topics;
    nh.getParam("o2ac_visualization/caminfo_topics", caminfo_topics);
    setCameraInfoCallbacks(nh, caminfo_topics);

    // Set callback functions for String topics
    XmlRpc::XmlRpcValue string_topics;
    nh.getParam("o2ac_visualization/string_topics", string_topics);
    setStringCallbacks(nh, string_topics);

    // Set callback function for kitting set id topics
    XmlRpc::XmlRpcValue kitting_set_id_topics;
    nh.getParam("o2ac_visualization/kitting_set_id_topics",
                kitting_set_id_topics);
    setKittingSetIdCallbacks(nh, kitting_set_id_topics);

    // Set callback function for suction success topics
    XmlRpc::XmlRpcValue suction_success_topics;
    nh.getParam("o2ac_visualization/suction_success_topics",
                suction_success_topics);
    setSuctionSuccessCallbacks(nh, suction_success_topics);

    // Set callback function for run mode topics
    XmlRpc::XmlRpcValue run_mode_topics;
    nh.getParam("o2ac_visualization/run_mode_topics", run_mode_topics);
    setRunModeCallbacks(nh, run_mode_topics);

    // Timer callback to keep this instance alive
    timer = nh.createTimer(ros::Duration(1), timerCallback);
  }

  bool resetTimer(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res) {
    start_time = now();
    res.success = true;
    ROS_INFO("Debug monitor timer reset");
    return true;
  }

private:
  std::vector<image_transport::Subscriber> i_sub_rs_;
  std::vector<ros::Subscriber> n_sub_rs_;
  std::vector<ImageCallback> imgcbs_;
  std::vector<CamInfoCallback> caminfocbs_;
  std::vector<StringCallback> stringcbs_;
  std::vector<KittingSetIdCallback> setidcbs_;
  std::vector<SuctionSuccessCallback> sscbs_;
  std::vector<RunModeCallback> rmcbs_;

  ros::Timer timer;

  cv::Mat monitor_;

  std::vector<std::string> suction_success_str;

  int width;
  int height;
  int n_cols;
  int n_rows;
  int d_cols;
  int d_rows;
  long start_time;

  cv::Rect getRect(XmlRpc::XmlRpcValue &params) {
    int col_min = params["col"][0];
    int col_max = params["col"][1];
    int row_min = params["row"][0];
    int row_max = params["row"][1];

    int x = col_min * d_cols;
    int y = row_min * d_rows;
    int w = (col_max - col_min + 1) * d_cols - 1;
    int h = (row_max - row_min + 1) * d_rows - 1;

    ROS_INFO("%d, %d, %d, %d", x, y, w, h);

    return cv::Rect(x, y, w, h);
  }

  void setGlobalParams(XmlRpc::XmlRpcValue &params) {
    width = params["width"];
    height = params["height"];
    n_cols = params["n_cols"];
    n_rows = params["n_rows"];
    d_cols = width / n_cols;
    d_rows = height / n_rows;
  }

  void setImageCallbacks(ros::NodeHandle &nh,
                         XmlRpc::XmlRpcValue image_topics) {
    image_transport::ImageTransport it_(nh);

    for (auto it = image_topics.begin(); it != image_topics.end(); it++) {
      auto params = image_topics[it->first];
      std::string topic_name = params["topic_name"];
      ROS_INFO("%s", topic_name.c_str());

      // Area in the debug monitor
      cv::Rect rect = getRect(params);

      // Create, set and keep callback function
      imgcbs_.push_back(getCallbackForImage(rect, params, monitor_));
      i_sub_rs_.push_back(it_.subscribe(topic_name, 1, imgcbs_.back()));
    }
  }

  void setCameraInfoCallbacks(ros::NodeHandle &nh,
                              XmlRpc::XmlRpcValue caminfo_topics) {
    for (auto it = caminfo_topics.begin(); it != caminfo_topics.end(); it++) {
      auto params = caminfo_topics[it->first];
      std::string topic_name = params["topic_name"];
      ROS_INFO("%s", topic_name.c_str());

      // Area in the debug monitor
      cv::Rect rect = getRect(params);

      // Create, set and keep callback function
      caminfocbs_.push_back(getCallbackForCameraInfo(rect, params, monitor_));
      n_sub_rs_.push_back(nh.subscribe(topic_name, 1, caminfocbs_.back()));
    }
  }

  void setStringCallbacks(ros::NodeHandle &nh,
                          XmlRpc::XmlRpcValue string_topics) {
    for (auto it = string_topics.begin(); it != string_topics.end(); it++) {
      auto params = string_topics[it->first];
      std::string topic_name = params["topic_name"];
      ROS_INFO("%s", topic_name.c_str());

      // Area in the debug monitor
      cv::Rect rect = getRect(params);

      // Create, set and keep callback function
      stringcbs_.push_back(
          getCallbackForString(rect, params, monitor_, start_time));
      n_sub_rs_.push_back(nh.subscribe(topic_name, 1, stringcbs_.back()));
    }
  }

  void setKittingSetIdCallbacks(ros::NodeHandle &nh,
                                XmlRpc::XmlRpcValue kitting_set_id_topics) {
    auto topics = kitting_set_id_topics;
    for (auto it = topics.begin(); it != topics.end(); it++) {
      auto params = topics[it->first];
      std::string topic_name = params["topic_name"];
      ROS_INFO("%s", topic_name.c_str());

      // Area in the debug monitor
      cv::Rect rect = getRect(params);

      // Create, set and keep callback function
      setidcbs_.push_back(
          getCallbackForKittingSetId(rect, params, monitor_, nh));
      n_sub_rs_.push_back(nh.subscribe(topic_name, 1, setidcbs_.back()));
    }
  }

  void setSuctionSuccessCallbacks(ros::NodeHandle &nh,
                                  XmlRpc::XmlRpcValue topics) {
    for (auto it = topics.begin(); it != topics.end(); it++) {
      auto params = topics[it->first];
      std::string topic_name = params["topic_name"];
      ROS_INFO("%s", topic_name.c_str());

      // Area in the debug monitor
      cv::Rect rect = getRect(params);

      // Create, set and keep callback function
      sscbs_.push_back(getCallbackForSuctionSuccess(rect, params, monitor_,
                                                    suction_success_str));
      n_sub_rs_.push_back(nh.subscribe(topic_name, 1, sscbs_.back()));
    }
  }

  void setRunModeCallbacks(ros::NodeHandle &nh, XmlRpc::XmlRpcValue topics) {
    for (auto it = topics.begin(); it != topics.end(); it++) {
      auto params = topics[it->first];
      std::string topic_name = params["topic_name"];
      ROS_INFO("%s", topic_name.c_str());

      // Area in the debug monitor
      cv::Rect rect = getRect(params);

      // Create, set and keep callback function
      rmcbs_.push_back(getCallbackForRunMode(rect, params, monitor_));
      n_sub_rs_.push_back(nh.subscribe(topic_name, 1, rmcbs_.back()));
    }
  }
};

class o2acDebugMonitor : public nodelet::Nodelet {
public:
  o2acDebugMonitor() {}
  ~o2acDebugMonitor() {}
  boost::shared_ptr<Monitor> inst_;
  Monitor *monitor;
  ros::ServiceServer srv_resettimer;

private:
  virtual void onInit() {
    auto nh = getNodeHandle();
    monitor = new Monitor(getNodeHandle());
    inst_.reset(monitor);

    // Add service ResetTimer
    srv_resettimer = nh.advertiseService("/o2ac_visualization/reset_timer",
                                         &Monitor::resetTimer, monitor);
  }
};

PLUGINLIB_EXPORT_CLASS(o2ac_visualization::o2acDebugMonitor, nodelet::Nodelet)
} // namespace o2ac_visualization
