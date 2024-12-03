#include "wpc_app_bridge/wpc_app_bridge.hpp"

namespace wpc_app_bridge
{
    WpcAppBridge::WpcAppBridge(const rclcpp::NodeOptions&option) : Node("WpcAppBridge", option)
    {
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/wpc_app_bridge/current",
            0,
            std::bind(&WpcAppBridge::current_callback, this, _1)
        );

        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/to_wpc", 0);

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/from_wpc",
            0,
            std::bind(&WpcAppBridge::target_callback, this, _1)
        );

        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/wpc_app_bridge/target", 0);
    }

    void WpcAppBridge::current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        tf2::Vector3 v;
        v.setW(msg->pose.orientation.w);
        v.setX(msg->pose.orientation.x);
        v.setY(msg->pose.orientation.y);
        v.setZ(msg->pose.orientation.z);

        geometry_msgs::msg::Point p;
        p.x = msg->pose.position.x;
        p.y = msg->pose.position.y;
        p.z = v.getZ();

        current_pose_pub_->publish(p);
    }

    void WpcAppBridge::target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        tf2::Quaternion q;
        q.setEuler(0.0, 0.0, msg->z);

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = msg->x;
        pose.pose.position.y = msg->y;
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        target_pose_pub_->publish(pose);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(wpc_app_bridge::WpcAppBridge)