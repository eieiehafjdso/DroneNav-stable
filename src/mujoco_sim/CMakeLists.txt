cmake_minimum_required(VERSION 3.8)
project(mujoco_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ROS2 实时渲染 viewer（订阅 /fmu/out/vehicle_odometry）
install(PROGRAMS src/view_ros.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME view_ros
)

# 接触 relay（把模块的 UDP 接触转成 /mujoco/contacts）
install(PROGRAMS src/px4_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME px4_bridge
)

install(PROGRAMS src/sim_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_node
)

install(PROGRAMS src/contact_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME contact_relay
)

ament_package()
