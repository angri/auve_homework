#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"


namespace traffic_light_analysis {

class AnalysisActor {
 public:
  explicit AnalysisActor(
    std::function<void(const std_msgs::Float32ConstPtr&)> zone_height_cb)
    : zone_height_cb_(zone_height_cb)
  {}

  void OnDetected(const std_msgs::BoolConstPtr& msg);
  void OnSize(const geometry_msgs::Vector3ConstPtr& msg);

 private:
  std::function<void(const std_msgs::Float32ConstPtr&)> zone_height_cb_;
};

}  // namespace traffic_light_analysis
