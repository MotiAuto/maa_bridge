cmake_minimum_required(VERSION 3.5)
project(wpc_app_bridge)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_wpc_app_bridge SHARED
  DIRECTORY src
)
target_include_directories(target_wpc_app_bridge
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_wpc_app_bridge
  PLUGIN "wpc_app_bridge::WpcAppBridge"
  EXECUTABLE wpc_app_bridge_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)