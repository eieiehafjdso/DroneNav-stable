cmake_minimum_required(VERSION 3.8)
project(drone_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# 键盘手动遥控（offboard 姿态+推力；碰撞恢复期间转发 /recovery/setpoint）
# ---------------------------------------------------------------------------
add_executable(drone_keyboard_teleop src/drone_keyboard_teleop.cpp)
target_include_directories(drone_keyboard_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(drone_keyboard_teleop Eigen3::Eigen)
ament_target_dependencies(drone_keyboard_teleop
  rclcpp px4_msgs std_msgs)
install(TARGETS drone_keyboard_teleop DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dq_math test/test_dq_math.cpp)
  target_include_directories(test_dq_math PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_dq_math Eigen3::Eigen)
endif()

ament_package()
