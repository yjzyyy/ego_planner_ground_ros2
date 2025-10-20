# <!-- <launch>
#   <!-- size of map, change the size inflate x, y, z according to your application -->
#   <arg name="map_size_x" value="300"/>
#   <arg name="map_size_y" value="300"/>
#   <arg name="map_size_z" value=" 4.0"/>

#   <!-- topic of your odometry such as VIO or LIO -->
#   <arg name="odom_topic" value="/state_estimation" />
#   <arg name="pointcloud_topic" value="/registered_scan" />

#   <!-- main algorithm params -->
#   <include file="$(find ego_planner)/launch/advanced_param.xml">

#     <arg name="map_size_x_" value="$(arg map_size_x)"/>
#     <arg name="map_size_y_" value="$(arg map_size_y)"/>
#     <arg name="map_size_z_" value="$(arg map_size_z)"/>
#     <arg name="odometry_topic" value="$(arg odom_topic)"/>

#     <!-- camera pose: transform of camera frame in the world frame -->
#     <!-- depth topic: depth image, 640x480 by default -->
#     <!-- don't set cloud_topic if you already set these ones! -->
#     <arg name="camera_pose_topic" value="/pcl_render_node/camera_pose"/>
#     <arg name="depth_topic" value="/pcl_render_node/depth"/>

#     <!-- topic of point cloud measurement, such as from LIDAR  -->
#     <!-- don't set camera pose and depth, if you already set this one! -->
#     <arg name="cloud_topic" value="$(arg pointcloud_topic)"/>
#     <arg name="goal_topic" value="/move_base_simple/goal"/>

#     <!-- intrinsic params of the depth camera -->

#     <arg name="cx" value="321.04638671875"/>
#     <arg name="cy" value="243.44969177246094"/>
#     <arg name="fx" value="387.229248046875"/>
#     <arg name="fy" value="387.229248046875"/>

#     <!-- maximum velocity and acceleration the drone will reach -->
#     <arg name="max_vel" value="1.5" />
#     <arg name="max_acc" value="3" />

#     <!--always set to 1.5 times grater than sensing horizen-->
#     <arg name="planning_horizon" value="10.5" />

#     <!-- 1: use 2D Nav Goal to select goal  -->
#     <!-- 2: use global waypoints below  -->
#     <arg name="flight_type" value="1" />
    
#     <!-- global waypoints -->
#     <!-- It generates a piecewise min-snap traj passing all waypoints -->
#     <arg name="point_num" value="5" />

#     <arg name="point0_x" value="-15.0" />
#     <arg name="point0_y" value="0.0" />
#     <arg name="point0_z" value="1.0" />

#     <arg name="point1_x" value="0.0" />
#     <arg name="point1_y" value="15.0" />
#     <arg name="point1_z" value="1.0" />

#     <arg name="point2_x" value="15.0" />
#     <arg name="point2_y" value="0.0" />
#     <arg name="point2_z" value="1.0" />

#     <arg name="point3_x" value="0.0" />
#     <arg name="point3_y" value="-15.0" />
#     <arg name="point3_z" value="1.0" />

#     <arg name="point4_x" value="-15.0" />
#     <arg name="point4_y" value="0.0" />
#     <arg name="point4_z" value="1.0" />
    
#   </include>

#   <!-- trajectory server -->

#   <node pkg="ego_planner" name="traj_server" type="traj_server" output="screen">
#     <remap from="/position_cmd" to="planning/pos_cmd"/>

#     <remap from="/odom_world" to="$(arg odom_topic)"/>
#     <param name="traj_server/time_forward" value="1.0" type="double"/>
#   </node>

#   <rosparam command="load" file="$(find patchwork)/config/params_ouster128.yaml" />

#   <node name="$(anon offline_own_data)" pkg="patchwork" type="offline_own_data" output="screen">
#   <rosparam param="/algorithm">"patchwork"</rosparam> 
#   <remap from="/pointcloud_scan" to="$(arg pointcloud_topic)"/>
#   </node>

#   <include file="$(find ego_planner)/launch/rviz.launch"/>




# </launch> -->










from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction 
import os


def generate_launch_description():
    # ====================== 1. 声明启动参数（对应原 launch 中的 <arg>）======================
    # 地图尺寸参数
    map_size_x_arg = DeclareLaunchArgument(
        name="map_size_x",
        default_value="300.0",
        description="Size of map in X direction"
    )
    map_size_y_arg = DeclareLaunchArgument(
        name="map_size_y",
        default_value="300.0",
        description="Size of map in Y direction"
    )
    map_size_z_arg = DeclareLaunchArgument(
        name="map_size_z",
        default_value="4.0",
        description="Size of map in Z direction"
    )

    # 话题参数（里程计、点云）
    odom_topic_arg = DeclareLaunchArgument(
        name="odom_topic",
        default_value="/state_estimation",
        description="Topic name of odometry (e.g., VIO/LIO)"
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        name="pointcloud_topic",
        default_value="/registered_scan",
        description="Topic name of pointcloud (e.g., from LIDAR)"
    )

    # ====================== 2. 包含 ego_planner 的 advanced_param 配置（对应原 <include>）======================
    # 找到 ego_planner 包的路径（ROS 2 用 FindPackageShare 替代 $(find)）
    ego_planner_share = FindPackageShare(package="ego_planner")
    advanced_param_launch = IncludeLaunchDescription(
        # 路径格式：包名/share/包名/launch/advanced_param.xml（需确认原 XML 是否适配 ROS 2，若未适配需同步转换该 XML）
        launch_description_source=PathJoinSubstitution(
            [ego_planner_share, "launch", "advanced_param.launch.py"]
        ),
        # 传递参数给被包含的启动文件（对应原 <arg name="xxx" value="$(arg yyy)"/>）
        launch_arguments={
            "map_size_x_": LaunchConfiguration("map_size_x"),
            "map_size_y_": LaunchConfiguration("map_size_y"),
            "map_size_z_": LaunchConfiguration("map_size_z"),
            "odometry_topic": LaunchConfiguration("odom_topic"),
            "camera_pose_topic": "/pcl_render_node/camera_pose",
            "depth_topic": "/pcl_render_node/depth",
            "cloud_topic": LaunchConfiguration("pointcloud_topic"),
            "goal_topic": "/move_base_simple/goal",
            "cx": "321.04638671875",
            "cy": "243.44969177246094",
            "fx": "387.229248046875",
            "fy": "387.229248046875",
            "max_vel": "1.5",
            "max_acc": "3.0",
            "planning_horizon": "10.5",
            "flight_type": "1",
            "point_num": "5",
            "point0_x": "-15.0",
            "point0_y": "0.0",
            "point0_z": "1.0",
            "point1_x": "0.0",
            "point1_y": "15.0",
            "point1_z": "1.0",
            "point2_x": "15.0",
            "point2_y": "0.0",
            "point2_z": "1.0",
            "point3_x": "0.0",
            "point3_y": "-15.0",
            "point3_z": "1.0",
            "point4_x": "-15.0",
            "point4_y": "0.0",
            "point4_z": "1.0"
        }.items()
    )

    # ====================== 3. 启动 traj_server 节点（对应原 <node>）======================
    traj_server_node = Node(
        package="ego_planner",          # 包名（不变）
        executable="traj_server",       # 可执行文件名（不变，需确保 ROS 2 编译生成该节点）
        name="traj_server",             # 节点名（不变）
        output="screen",                # 日志输出到终端（不变）
        # 话题重映射（对应原 <remap>）
        remappings=[
            ("/position_cmd", "planning/pos_cmd"),
            ("/odom_world", LaunchConfiguration("odom_topic"))
            # ("/odom_world", "/state_estimation")
        ],
        # 节点参数（对应原 <param>）
        parameters=[
            {"traj_server/time_forward": 1.0,
            "MPC/v_max": 1.8,
            "MPC/w_max": 1.2,
            "MPC/omega0": 1.0,
            "MPC/omega1": 0.1
            }]
    )

    # ====================== 4. 加载 patchwork 的参数文件（对应原 <rosparam load>）======================
    # 找到 patchwork 包的 config 路径
    patchwork_share = get_package_share_directory("patchwork")
    patchwork_params_path = os.path.join(patchwork_share, "config", "ouster128.yaml")

    # 启动 patchwork 节点（对应原 <node> + <rosparam>）
    patchwork_node = Node(
        package="patchwork",
        executable="offline_own_data",
        name="offline_own_data",  # ROS 2 不推荐 $(anon)，若需匿名可删除 name 让系统自动分配
        output="screen",
        # 加载 YAML 参数文件 + 单独设置参数
        parameters=[
            patchwork_params_path,  # 先加载 YAML 文件
            {"/algorithm": "patchwork"}  # 单独设置的参数（覆盖 YAML 中同名参数）
        ],
        # 话题重映射
        remappings=[
            ("/pointcloud_scan", LaunchConfiguration("pointcloud_topic"))
        ]
    )

    # ====================== 5. 包含 rviz 启动文件（对应原 <include rviz.launch>）======================
    rviz_launch = IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution(
            [ego_planner_share, "launch", "rviz.launch.py"]  # 注意：ROS 2 需将原 rviz.launch 转为 rviz.launch.py
        )
    )

    # ====================== 6. 组装所有启动项并返回 ======================

    main_group = GroupAction(
        actions=[
            # 包含 advanced_param（依赖 odom_topic）
            advanced_param_launch,
            # 启动 traj_server（依赖 odom_topic）
            traj_server_node,
            # 启动 patchwork（依赖 pointcloud_topic）
            patchwork_node,
            # 启动 RViz（无强依赖，可放最后）
            rviz_launch
        ]
    )

    return LaunchDescription([
        # 1. 声明的参数
        map_size_x_arg,
        map_size_y_arg,
        map_size_z_arg,
        odom_topic_arg,
        pointcloud_topic_arg,
        main_group
    ])

