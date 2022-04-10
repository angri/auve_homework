#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"

#include "trt_ssd.hpp"


namespace traffic_light_fetcher {

class FetcherActor {
 public:
  FetcherActor(
      const std::string& onnx_file,
      const std::string& label_file,
      std::function<void(const sensor_msgs::ImageConstPtr&)> vis_cb,
      std::function<void(const std_msgs::BoolConstPtr&)> detected_cb,
      std::function<void(const geometry_msgs::Vector3ConstPtr&)> size_cb);

  void OnImage(const sensor_msgs::ImageConstPtr& msg);

  bool IsReady() const {
    return ready_;
  };

 private:
  std::function<void(const sensor_msgs::ImageConstPtr&)> vis_cb_;
  std::function<void(const std_msgs::BoolConstPtr&)> detected_cb_;
  std::function<void(const geometry_msgs::Vector3ConstPtr&)> size_cb_;

  ros::Time last_incoming_image_ = ros::TIME_MIN;

  static constexpr double kMinInterval = 1.0;  // sec
  static constexpr float kProbThreshold = 0.07;

  std::unique_ptr<ssd::Net> net_ptr_;
  int tlr_id_;
  int channel_;
  int width_;
  int height_;
  int class_num_;
  int detection_per_class_;

  bool ready_ = false;
};

}  // namespace traffic_light_fetcher
