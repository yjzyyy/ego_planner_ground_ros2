cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(path_searching REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)


include_directories( 
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

ament_export_include_directories(include)

add_library( bspline_opt 
    src/uniform_bspline.cpp 
    src/bspline_optimizer.cpp 
    src/gradient_descent_optimizer.cpp
    )
ament_target_dependencies(bspline_opt 
                          std_msgs visualization_msgs 
                          nav_msgs cv_bridge 
                          rclcpp Eigen3 
                          PCL pcl_conversions 
                          plan_env  path_searching)

install(TARGETS bspline_opt
  DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(bspline_opt)

ament_export_dependencies(plan_env) 

ament_package()