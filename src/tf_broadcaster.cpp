#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TFBroadcaster : public rclcpp::Node
{
public:
  TFBroadcaster()
  : Node("tf_broadcaster")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Create a timer that fires every 100ms
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TFBroadcaster::timer_callback, this));
  }

private:
  void timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    double time_sec = now.seconds();

    geometry_msgs::msg::TransformStamped t;

    // Set header and frame IDs
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = "robot";

    // Set translation for circle: x = 2.0 * cos(time), y = 2.0 * sin(time)
    t.transform.translation.x = 2.0 * cos(time_sec);
    t.transform.translation.y = 2.0 * sin(time_sec);
    t.transform.translation.z = 0.0;

    // Set rotation to identity (no rotation)
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
