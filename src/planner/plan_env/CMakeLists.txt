cmake_minimum_required(VERSION 3.8)
project(plan_env)

# 默认设置为Release模式
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# 查找ROS2包
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)  # 通常cv_bridge需要sensor_msgs
find_package(nav_msgs REQUIRED)

# 包含目录
include_directories(
    SYStEM
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})

ament_export_include_directories(include)

# 创建库
add_library(plan_env SHARED
  src/grid_map.cpp
  src/raycast.cpp
)

# 链接库
target_link_libraries(plan_env
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(plan_env std_msgs visualization_msgs geometry_msgs nav_msgs rclcpp Eigen3 PCL cv_bridge message_filters pcl_conversions
)


# 安装目标
install(TARGETS plan_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_export_dependencies(
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  Eigen3
  PCL
  OpenCV
)

# 添加测试（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()