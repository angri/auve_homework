#include "ros/ros.h"
#include "image_transport/image_transport.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"

#include "fetcher.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traffic_light_fetcher");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  std::string onnx_file;
  std::string label_file;
  nh.getParam("onnx_file", onnx_file);
  nh.getParam("label_file", label_file);

  ros::Publisher vis_pub = nh.advertise<sensor_msgs::Image>("/traffic_light_vis", 1);
  ros::Publisher detected_pub = nh.advertise<std_msgs::Bool>("/traffic_light_detected", 1);
  ros::Publisher size_pub = nh.advertise<geometry_msgs::Vector3>("/traffic_light_size", 1);

  traffic_light_fetcher::FetcherActor fetcher(
      onnx_file,
      label_file,
      [&vis_pub] (const sensor_msgs::ImageConstPtr& msg) { vis_pub.publish(msg); },
      [&detected_pub] (const std_msgs::BoolConstPtr& msg) { detected_pub.publish(msg); },
      [&size_pub] (const geometry_msgs::Vector3ConstPtr& msg) { size_pub.publish(msg); });

  image_transport::Subscriber image_sub = it.subscribe(
      "/image_raw", 1, &traffic_light_fetcher::FetcherActor::OnImage, &fetcher);

  if (!fetcher.IsReady()) {
    return 1;
  }
  ROS_INFO("we are ready");
  ros::spin();
  return 0;
}
