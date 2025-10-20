cmake_minimum_required(VERSION 3.8)
project(traj_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

rosidl_generate_interfaces(traj_utils
  "msg/Bspline.msg"
  "msg/DataDisp.msg"

  DEPENDENCIES std_msgs geometry_msgs rclcpp builtin_interfaces
)


include_directories( 
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

ament_export_include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils_lib SHARED
    src/planning_visualization.cpp 
    src/polynomial_traj.cpp 
    )



ament_target_dependencies(traj_utils_lib 
std_msgs geometry_msgs visualization_msgs rclcpp Eigen3 PCL)

install(TARGETS traj_utils_lib
        DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(traj_utils_lib)

ament_package()