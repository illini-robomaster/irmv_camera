// MindVision camera SDK
#include <CameraApi.h>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++ standard library
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace mindvision_camera {

// thread synchronization variables
std::mutex m;
std::condition_variable trigger;

// global variables for image processing
uint8_t *pby_buffer_;
tSdkFrameHead s_frame_info_;

// FPS calculation init
auto last_time = std::chrono::high_resolution_clock::now();
auto print_last_time = std::chrono::high_resolution_clock::now();
float fps = 0;

// callback function for image processing
void getImage(CameraHandle hCamera, BYTE *pFrameBuffer,
              tSdkFrameHead *pFrameHead, PVOID pContext) {
  std::unique_lock<std::mutex> lock(m);
  pby_buffer_ = pFrameBuffer;
  s_frame_info_ = *pFrameHead;
  trigger.notify_one();
}

// camera node
class MVCameraNode : public rclcpp::Node {
 public:
  explicit MVCameraNode(const rclcpp::NodeOptions &options)
      : Node("mv_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode!");

    CameraSdkInit(0);

    int i_camera_counts = 1;
    int i_status = -1;
    tSdkCameraDevInfo t_camera_enum_list;
    i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
    RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

    if (i_camera_counts == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      return;
    }

    i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

    RCLCPP_INFO(this->get_logger(), "Init state = %d", i_status);
    if (i_status != CAMERA_STATUS_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Init failed!");
      return;
    }

    // set trigger mode to be hardware trigger
    CameraSetTriggerMode(h_camera_, 2);

    CameraGetCapability(h_camera_, &t_capability_);
    image_msg_.data.reserve(t_capability_.sResolutionRange.iHeightMax *
                            t_capability_.sResolutionRange.iWidthMax * 3);
    CameraSetAeState(h_camera_, false);
    declareParameters();
    CameraPlay(h_camera_);
    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8);

    // create camera publisher
    // rqt_image_view can't subscribe image msg with sensor_data QoS
    // https://github.com/ros-visualization/rqt/issues/187
    bool use_sensor_data_qos =
        this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                   : rmw_qos_profile_default;
    // publisher
    camera_pub_ =
        image_transport::create_camera_publisher(this, "image_raw", qos);

    // load camera info
    camera_name_ = this->declare_parameter("camera_name", "mv_camera");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this,
                                                                 camera_name_);
    auto camera_info_url = this->declare_parameter(
        "camera_info_url",
        "package://mindvision_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                  camera_info_url.c_str());

    // add callback to the set parameter event
    params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &MVCameraNode::parametersCallback, this, std::placeholders::_1));

    // verify trigger mode
    int trigger_mode_;
    CameraGetTriggerMode(h_camera_, &trigger_mode_);
    RCLCPP_INFO(this->get_logger(), "Trigger mode = %d", trigger_mode_);

    // register callback function
    CameraSetCallbackFunction(h_camera_, &getImage, nullptr, nullptr);

    // define and start capture thread
    capture_thread_ = std::thread{[this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      std::unique_lock<std::mutex> lock(m);
      while (rclcpp::ok()) {
        // wait for hardware trigger
        trigger.wait(lock);

        // calculate FPS
        auto current_time = std::chrono::high_resolution_clock::now();
        fps = 1.0 /
              std::chrono::duration<double>(current_time - last_time).count();
        last_time = current_time;
        if (std::chrono::duration<double>(current_time - print_last_time)
                .count() > 0.3) {
          RCLCPP_INFO(this->get_logger(), "Hz: %6.2f", fps);
          print_last_time = current_time;
        }

        // process image
        CameraImageProcess(h_camera_, pby_buffer_, image_msg_.data.data(),
                           &s_frame_info_);
        if (flip_image_)
          CameraFlipFrameBuffer(image_msg_.data.data(), &s_frame_info_, 3);
        camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
        image_msg_.height = s_frame_info_.iHeight;
        image_msg_.width = s_frame_info_.iWidth;
        image_msg_.step = s_frame_info_.iWidth * 3;
        image_msg_.data.resize(s_frame_info_.iWidth * s_frame_info_.iHeight *
                               3);

        // publish image
        camera_pub_.publish(image_msg_, camera_info_msg_);
      }
    }};
  }

  ~MVCameraNode() override {
    if (capture_thread_.joinable()) capture_thread_.join();
    CameraUnInit(h_camera_);
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
  }

 private:
  // declare parameters
  void declareParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // exposure time
    param_desc.description = "Exposure time in microseconds";
    double exposure_line_time;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    param_desc.integer_range[0].from_value =
        t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
    param_desc.integer_range[0].to_value =
        t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;
    double exposure_time =
        this->declare_parameter("exposure_time", 5000, param_desc);
    CameraSetExposureTime(h_camera_, exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);

    // analog gain
    param_desc.description = "Analog gain";
    param_desc.integer_range[0].from_value =
        t_capability_.sExposeDesc.uiAnalogGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sExposeDesc.uiAnalogGainMax;
    int analog_gain;
    CameraGetAnalogGain(h_camera_, &analog_gain);
    analog_gain =
        this->declare_parameter("analog_gain", analog_gain, param_desc);
    CameraSetAnalogGain(h_camera_, analog_gain);
    RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

    // RGB gain
    // get default value
    CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
    // R gain
    param_desc.integer_range[0].from_value =
        t_capability_.sRgbGainRange.iRGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sRgbGainRange.iRGainMax;
    r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_desc);
    // G gain
    param_desc.integer_range[0].from_value =
        t_capability_.sRgbGainRange.iGGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sRgbGainRange.iGGainMax;
    g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_desc);
    // B gain
    param_desc.integer_range[0].from_value =
        t_capability_.sRgbGainRange.iBGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sRgbGainRange.iBGainMax;
    b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_desc);
    // set gain
    CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: R = %d", r_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: G = %d", g_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: B = %d", b_gain_);

    // saturation
    param_desc.description = "Saturation";
    param_desc.integer_range[0].from_value =
        t_capability_.sSaturationRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
    int saturation;
    CameraGetSaturation(h_camera_, &saturation);
    saturation = this->declare_parameter("saturation", saturation, param_desc);
    CameraSetSaturation(h_camera_, saturation);
    RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

    // gamma
    param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
    int gamma;
    CameraGetGamma(h_camera_, &gamma);
    gamma = this->declare_parameter("gamma", gamma, param_desc);
    CameraSetGamma(h_camera_, gamma);
    RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);

    // flip
    flip_image_ = this->declare_parameter("flip_image", false);
  }

  // callback function for set parameter event
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = CameraSetExposureTime(h_camera_, param.as_int());
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "analog_gain") {
        int status = CameraSetAnalogGain(h_camera_, param.as_int());
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failed to set analog gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.r") {
        r_gain_ = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failed to set RGB gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.g") {
        g_gain_ = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failed to set RGB gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.b") {
        b_gain_ = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failed to set RGB gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "saturation") {
        int status = CameraSetSaturation(h_camera_, param.as_int());
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failed to set saturation, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gamma") {
        int gamma = param.as_int();
        int status = CameraSetGamma(h_camera_, gamma);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failed to set Gamma, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "flip_image")
        flip_image_ = param.as_bool();
      else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  int h_camera_;
  tSdkCameraCapbility t_capability_;
  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;
  int r_gain_, g_gain_, b_gain_;  // RGB gain
  bool flip_image_;
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::thread capture_thread_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace mindvision_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MVCameraNode)
