#include "ros/ros.h"

#include "analysis.h"


namespace traffic_light_analysis {

void AnalysisActor::OnDetected(const std_msgs::BoolConstPtr& msg) {
  ROS_INFO(
      "received 'traffic light detected' message with detected = '%s'",
      msg->data ? "true" : "false");
}

void AnalysisActor::OnSize(const geometry_msgs::Vector3ConstPtr& msg) {
  auto zone_height = boost::make_shared<std_msgs::Float32>();
  zone_height->data = msg->y / 3;
  ROS_INFO("zone height = %.1f", zone_height->data);
  zone_height_cb_(zone_height);
}

}  // namespace traffic_light_analysis
