cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

# 输出IPOPT相关编译参数（保持原功能）
message(STATUS "IPOPT_CFLAGS: " ${IPOPT_CFLAGS})
message(STATUS "IPOPT_CFLAGS_OTHER: " ${IPOPT_CFLAGS_OTHER})

# 设置C++编译选项（ROS 2推荐C++14及以上标准）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_CSTDDEF -DHAVE_MPI_INITIALIZED -Wall -Wextra")

# 查找依赖包（ROS 2版本）
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 

# 声明库目标
add_library(mpc_controller
    src/MPC.cpp
)

# 设置库的包含目录
target_include_directories(mpc_controller
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    /usr/local/include
    ${EIGEN3_INCLUDE_DIR}
    ${nav_msgs_INCLUDE_DIRS}        # 新增：nav_msgs 头文件目录
    ${geometry_msgs_INCLUDE_DIRS} 
)

# 链接库依赖（ROS 2使用ament_target_dependencies管理依赖）
ament_target_dependencies(mpc_controller
    rclcpp
    nav_msgs
    geometry_msgs
)

# 链接系统库（如qpOASES，保持原路径）
target_link_libraries(mpc_controller
    /usr/local/lib/libqpOASES.so
)

# 安装库文件
install(TARGETS mpc_controller
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


# 第一个可执行文件：qp_test
add_executable(qp_test
    app/qp_test.cpp
)
target_include_directories(qp_test
    PRIVATE
    include
    /usr/local/include
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(qp_test
    /usr/local/lib/libqpOASES.so
)
install(TARGETS qp_test
    DESTINATION lib/${PROJECT_NAME}
)

# 第二个可执行文件：mpc_test
add_executable(mpc_test
    app/mpc_control_test.cpp
    src/uniform_bspline.cpp
    src/MPC.cpp
)
target_include_directories(mpc_test
    PRIVATE
    include
    /usr/local/include
    ${EIGEN3_INCLUDE_DIR}
    ${nav_msgs_INCLUDE_DIRS}        # 新增：mpc_test 需要 nav_msgs 头文件
    ${geometry_msgs_INCLUDE_DIRS} 
)
ament_target_dependencies(mpc_test
    rclcpp
    nav_msgs
    geometry_msgs
)
target_link_libraries(mpc_test
    /usr/local/lib/libqpOASES.so
)
install(TARGETS mpc_test
    DESTINATION lib/${PROJECT_NAME}
)

# 确保外部项目能找到本包的头文件和库
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)  # 导出目标
ament_export_dependencies(rclcpp nav_msgs geometry_msgs Eigen3)  # 导出依赖
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(mpc_controller)

# ROS 2包必须的收尾宏
ament_package()