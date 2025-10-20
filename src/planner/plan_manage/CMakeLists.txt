cmake_minimum_required(VERSION 3.8)
project(ego_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(mpc_controller REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate messages in the 'msg' folder
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Bspline.msg"
#   "msg/DataDisp.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )

include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../mpc_controller/include
  ${PROJECT_SOURCE_DIR}/../traj_utils/include
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )


# 添加依赖
ament_target_dependencies(ego_planner_node 
                          rclcpp nav_msgs visualization_msgs 
                          geometry_msgs std_msgs
                          plan_env path_searching 
                          bspline_opt traj_utils 
                          cv_bridge pcl_conversions mpc_controller
                          tf2 tf2_ros tf2_msgs tf2_geometry_msgs)

find_library(TRAJ_UTILS_LIB traj_utils_lib PATHS ${traj_utils_LIBRARY_DIRS})

target_link_libraries(ego_planner_node
    ${TRAJ_UTILS_LIB}
)

add_executable(traj_server src/traj_server.cpp)

ament_target_dependencies(traj_server 
                          rclcpp nav_msgs 
                          visualization_msgs geometry_msgs 
                          plan_env std_msgs mpc_controller
                          path_searching bspline_opt 
                          traj_utils cv_bridge pcl_conversions
                          tf2 tf2_ros tf2_msgs tf2_geometry_msgs)

target_link_libraries(traj_server
  libqpOASES.so  # 外部库
  )
                  

# Install targets
install(
  TARGETS ego_planner_node traj_server
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

