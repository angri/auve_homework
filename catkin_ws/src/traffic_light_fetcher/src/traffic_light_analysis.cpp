#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"

#include "analysis.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traffic_light_analysis");
  ros::NodeHandle nh("~");

  ros::Publisher zone_height_pub = nh.advertise<std_msgs::Float32>("/zone_height", 1);

  traffic_light_analysis::AnalysisActor analysis(
      [&zone_height_pub] (const std_msgs::Float32ConstPtr& msg) { zone_height_pub.publish(msg); });

  ros::Subscriber detected_sub = nh.subscribe(
      "/traffic_light_detected", 1, &traffic_light_analysis::AnalysisActor::OnDetected, &analysis);
  ros::Subscriber size_sub = nh.subscribe(
      "/traffic_light_size", 1, &traffic_light_analysis::AnalysisActor::OnSize, &analysis);

  ros::spin();
  return 0;
}
