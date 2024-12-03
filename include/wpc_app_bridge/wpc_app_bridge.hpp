#ifndef WPC_APP_BRIDGE_HPP_
#define WPC_APP_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using std::placeholders::_1;

namespace wpc_app_bridge
{
    class WpcAppBridge : public rclcpp::Node
    {
        public:
        explicit WpcAppBridge(const rclcpp::NodeOptions&option=rclcpp::NodeOptions());

        void current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void target_callback(const geometry_msgs::msg::Point::SharedPtr msg);

        private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_pose_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    };
}

#endif