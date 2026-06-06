cmake_minimum_required(VERSION 3.8)
project(argus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED) # 用 C++ 写 OpenCV，

# ==========================================================
# 1. 安装当前的 Python 节点与 Launch/Config 文件
# ==========================================================
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_info.py
  scripts/trigger.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================
# 编译 C++ 源码专用区域
# ==========================================================

# add_executable(my_algorithm_node src/my_algorithm.cpp)
# target_include_directories(my_algorithm_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(my_algorithm_node 
#   rclcpp 
#   sensor_msgs 
#   nav_msgs 
#   cv_bridge
# )
# install(TARGETS my_algorithm_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ==========================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()