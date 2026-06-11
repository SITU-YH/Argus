cmake_minimum_required(VERSION 3.8)
project(argus)

# 🌟 必须开启 C++17 以支持 std::filesystem
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# 🌟 解开 OpenCV 的注释
find_package(OpenCV REQUIRED) 

# ==========================================================
# 1. 安装 Launch/Config 文件与 Python 脚本
# ==========================================================
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 如果你决定彻底抛弃 Python 版本的 trigger，可以把下面这几行注释掉；
# 如果想留着备用也可以保持原样，因为 C++ 节点的名字叫 angle_trigger_node，它们不会冲突。
install(PROGRAMS
  scripts/trigger.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================
# 2. 编译 C++ 源码专用区域
# ==========================================================

# 声明可执行文件，指向我们刚写的 cpp 文件
add_executable(angle_trigger_node src/angle_trigger_node.cpp)

# 包含头文件目录 (将 OpenCV 加进来)
target_include_directories(angle_trigger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# 链接 ROS2 的核心库
ament_target_dependencies(angle_trigger_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  cv_bridge
)

# 链接 OpenCV 的动态库
target_link_libraries(angle_trigger_node ${OpenCV_LIBRARIES})

# 安装编译好的 C++ 节点
install(TARGETS angle_trigger_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
