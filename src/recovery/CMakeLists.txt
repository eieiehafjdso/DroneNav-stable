cmake_minimum_required(VERSION 3.8)
project(recovery)

find_package(ament_cmake REQUIRED)

# 常驻碰撞恢复节点（Python，直接 import dq_lib 里的论文实现）
install(PROGRAMS scripts/collision_recovery_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME collision_recovery_node)

# 对偶四元数参考实现
install(DIRECTORY dq_lib DESTINATION share/${PROJECT_NAME})

# 参数 + 启动文件
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
