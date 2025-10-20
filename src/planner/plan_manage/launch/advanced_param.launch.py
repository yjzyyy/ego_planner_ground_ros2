# <launch>
#   <arg name="map_size_x_"/>
#   <arg name="map_size_y_"/>
#   <arg name="map_size_z_"/>

#   <arg name="odometry_topic"/>
#   <arg name="goal_topic"/>
#   <arg name="camera_pose_topic"/>
#   <arg name="depth_topic"/>
#   <arg name="cloud_topic"/>

#   <arg name="cx"/>
#   <arg name="cy"/>
#   <arg name="fx"/>
#   <arg name="fy"/>

#   <arg name="max_vel"/>
#   <arg name="max_acc"/>
#   <arg name="planning_horizon"/>

#   <arg name="point_num"/>
#   <arg name="point0_x"/>
#   <arg name="point0_y"/>
#   <arg name="point0_z"/>
#   <arg name="point1_x"/>
#   <arg name="point1_y"/>
#   <arg name="point1_z"/>
#   <arg name="point2_x"/>
#   <arg name="point2_y"/>
#   <arg name="point2_z"/>
#   <arg name="point3_x"/>
#   <arg name="point3_y"/>
#   <arg name="point3_z"/>
#   <arg name="point4_x"/>
#   <arg name="point4_y"/>
#   <arg name="point4_z"/>

#   <arg name="flight_type"/>

#   <!-- main node -->
#   <node pkg="ego_planner" name="ego_planner_node" type="ego_planner_node" output="screen">
#     <remap from="/odom_map" to="$(arg odometry_topic)"/>
#     <remap from="/way_point" to="$(arg goal_topic)"/>
#     <remap from="/grid_map/odom" to="$(arg odometry_topic)"/>
#     <remap from="/grid_map/cloud" to="$(arg cloud_topic)"/>
#     <remap from = "/grid_map/pose"   to = "$(arg camera_pose_topic)"/> 
#     <remap from = "/grid_map/depth" to = "$(arg depth_topic)"/>

#     <!-- planning fsm -->
#     <param name="fsm/flight_type" value="$(arg flight_type)" type="int"/>
#     <param name="fsm/thresh_replan" value="1.5" type="double"/>
#     <param name="fsm/thresh_no_replan" value="2.0" type="double"/>
#     <param name="fsm/planning_horizon" value="$(arg planning_horizon)" type="double"/> <!--always set to 1.5 times grater than sensing horizen-->
#     <param name="fsm/planning_horizen_time" value="3" type="double"/>
#     <param name="fsm/emergency_time_" value="1.0" type="double"/>
#     <param name="fsm/w_adjust_" value="1.0" type="double"/>

#     <param name="fsm/waypoint_num" value="$(arg point_num)" type="int"/>
#     <param name="fsm/waypoint0_x" value="$(arg point0_x)" type="double"/>
#     <param name="fsm/waypoint0_y" value="$(arg point0_y)" type="double"/>
#     <param name="fsm/waypoint0_z" value="$(arg point0_z)" type="double"/>
#     <param name="fsm/waypoint1_x" value="$(arg point1_x)" type="double"/>
#     <param name="fsm/waypoint1_y" value="$(arg point1_y)" type="double"/>
#     <param name="fsm/waypoint1_z" value="$(arg point1_z)" type="double"/>
#     <param name="fsm/waypoint2_x" value="$(arg point2_x)" type="double"/>
#     <param name="fsm/waypoint2_y" value="$(arg point2_y)" type="double"/>
#     <param name="fsm/waypoint2_z" value="$(arg point2_z)" type="double"/>
#     <param name="fsm/waypoint3_x" value="$(arg point3_x)" type="double"/>
#     <param name="fsm/waypoint3_y" value="$(arg point3_y)" type="double"/>
#     <param name="fsm/waypoint3_z" value="$(arg point3_z)" type="double"/>
#     <param name="fsm/waypoint4_x" value="$(arg point4_x)" type="double"/>
#     <param name="fsm/waypoint4_y" value="$(arg point4_y)" type="double"/>
#     <param name="fsm/waypoint4_z" value="$(arg point4_z)" type="double"/>

#     <param name="grid_map/resolution"      value="0.1" />
#     <param name="grid_map/map_size_x"   value="$(arg map_size_x_)" />
#     <param name="grid_map/map_size_y"   value="$(arg map_size_y_)" />
#     <param name="grid_map/map_size_z"   value="$(arg map_size_z_)" />
#     <param name="grid_map/local_update_range_x"  value="8.0" />
#     <param name="grid_map/local_update_range_y"  value="8.0" />
#     <param name="grid_map/local_update_range_z"  value="1.5" />
#     <param name="grid_map/obstacles_inflation"     value="0.5" />
#     <param name="grid_map/local_map_margin" value="2"/>
#     <param name="grid_map/ground_height"        value="-0.01"/>
#     <!-- camera parameter -->
#     <param name="grid_map/cx" value="$(arg cx)"/>
#     <param name="grid_map/cy" value="$(arg cy)"/>
#     <param name="grid_map/fx" value="$(arg fx)"/>
#     <param name="grid_map/fy" value="$(arg fy)"/>
#     <!-- depth filter -->
#     <param name="grid_map/use_depth_filter" value="true"/>
#     <param name="grid_map/depth_filter_tolerance" value="0.15"/>
#     <param name="grid_map/depth_filter_maxdist"   value="5.0"/>
#     <param name="grid_map/depth_filter_mindist"   value="0.2"/>
#     <param name="grid_map/depth_filter_margin"    value="1"/>
#     <param name="grid_map/k_depth_scaling_factor" value="1000.0"/>
#     <param name="grid_map/skip_pixel" value="2"/>
#     <!-- local fusion -->
#     <param name="grid_map/p_hit"  value="0.65"/>
#     <param name="grid_map/p_miss" value="0.35"/>
#     <param name="grid_map/p_min"  value="0.12"/>
#     <param name="grid_map/p_max"  value="0.90"/>
#     <param name="grid_map/p_occ"  value="0.80"/>
#     <param name="grid_map/min_ray_length" value="0.1"/>
#     <param name="grid_map/max_ray_length" value="4.0"/>

#     <param name="grid_map/virtual_ceil_height"   value="2.5"/>
#     <param name="grid_map/visualization_truncate_height"   value="1.6"/>
#     <param name="grid_map/show_occ_time"  value="false"/>
#     <param name="grid_map/pose_type"     value="2"/>
#     <param name="grid_map/map_type"     value="1"/>
#     <param name="grid_map/frame_id"      value="map"/>

#   <!-- planner manager -->
#     <param name="manager/max_vel" value="$(arg max_vel)" type="double"/>
#     <param name="manager/max_acc" value="$(arg max_acc)" type="double"/>
#     <param name="manager/max_jerk" value="4" type="double"/>
#     <param name="manager/control_points_distance" value="0.6" type="double"/>
#     <param name="manager/feasibility_tolerance" value="0.05" type="double"/>
#     <param name="manager/planning_horizon" value="$(arg planning_horizon)" type="double"/>

#   <!-- trajectory optimization -->
#     <param name="optimization/lambda_smooth" value="1.0" type="double"/>
#     <param name="optimization/lambda_collision" value="0.8" type="double"/>
#     <param name="optimization/lambda_feasibility" value="0.1" type="double"/>
#     <param name="optimization/lambda_fitness" value="1.0" type="double"/>
#     <param name="optimization/dist0" value="1.0" type="double"/>
#     <param name="optimization/max_vel" value="$(arg max_vel)" type="double"/>
#     <param name="optimization/max_acc" value="$(arg max_acc)" type="double"/>

#     <param name="bspline/limit_vel" value="$(arg max_vel)" type="double"/>
#     <param name="bspline/limit_acc" value="$(arg max_acc)" type="double"/>
#     <param name="bspline/limit_ratio" value="1.1" type="double"/>

#     <param name="MPC/v_max" value="1.8" type="double"/>
#     <param name="MPC/w_max" value="1.2" type="double"/>
#     <param name="MPC/omega0" value="1.0" type="double"/>
#     <param name="MPC/omega1" value="0.1" type="double"/>

#   </node>

# </launch>









from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明所有输入参数
    args = [
        # 地图尺寸参数
        DeclareLaunchArgument('map_size_x_', description='Map size in X direction'),
        DeclareLaunchArgument('map_size_y_', description='Map size in Y direction'),
        DeclareLaunchArgument('map_size_z_', description='Map size in Z direction'),
        
        # 话题参数
        DeclareLaunchArgument('odometry_topic', description='Odometry topic name'),
        DeclareLaunchArgument('goal_topic', description='Goal topic name'),
        DeclareLaunchArgument('camera_pose_topic', description='Camera pose topic name'),
        DeclareLaunchArgument('depth_topic', description='Depth image topic name'),
        DeclareLaunchArgument('cloud_topic', description='Point cloud topic name'),
        
        # 相机内参
        DeclareLaunchArgument('cx', description='Camera cx parameter'),
        DeclareLaunchArgument('cy', description='Camera cy parameter'),
        DeclareLaunchArgument('fx', description='Camera fx parameter'),
        DeclareLaunchArgument('fy', description='Camera fy parameter'),
        
        # 运动参数
        DeclareLaunchArgument('max_vel', description='Maximum velocity'),
        DeclareLaunchArgument('max_acc', description='Maximum acceleration'),
        DeclareLaunchArgument('planning_horizon', description='Planning horizon distance'),
        
        # 航点参数
        DeclareLaunchArgument('point_num', description='Number of waypoints'),
        DeclareLaunchArgument('point0_x', description='Waypoint 0 X coordinate'),
        DeclareLaunchArgument('point0_y', description='Waypoint 0 Y coordinate'),
        DeclareLaunchArgument('point0_z', description='Waypoint 0 Z coordinate'),
        DeclareLaunchArgument('point1_x', description='Waypoint 1 X coordinate'),
        DeclareLaunchArgument('point1_y', description='Waypoint 1 Y coordinate'),
        DeclareLaunchArgument('point1_z', description='Waypoint 1 Z coordinate'),
        DeclareLaunchArgument('point2_x', description='Waypoint 2 X coordinate'),
        DeclareLaunchArgument('point2_y', description='Waypoint 2 Y coordinate'),
        DeclareLaunchArgument('point2_z', description='Waypoint 2 Z coordinate'),
        DeclareLaunchArgument('point3_x', description='Waypoint 3 X coordinate'),
        DeclareLaunchArgument('point3_y', description='Waypoint 3 Y coordinate'),
        DeclareLaunchArgument('point3_z', description='Waypoint 3 Z coordinate'),
        DeclareLaunchArgument('point4_x', description='Waypoint 4 X coordinate'),
        DeclareLaunchArgument('point4_y', description='Waypoint 4 Y coordinate'),
        DeclareLaunchArgument('point4_z', description='Waypoint 4 Z coordinate'),
        
        # 飞行类型
        DeclareLaunchArgument('flight_type', description='Flight type (1: 2D Nav Goal, 2: global waypoints)'),

        # 在 advanced_param.launch.py 中添加
        DeclareLaunchArgument(
            "odometry_topic",
            default_value="/state_estimation",
            description="Odometry topic for ego_planner"
        )

    ]
    
    # 创建ego_planner_node节点
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        # 话题重映射
        remappings=[
            ('/odom_map', LaunchConfiguration('odometry_topic')),
            ('/way_point', LaunchConfiguration('goal_topic')),
            ('/grid_map/odom', LaunchConfiguration('odometry_topic')),
            ('/grid_map/cloud', LaunchConfiguration('cloud_topic')),
            ('/grid_map/pose', LaunchConfiguration('camera_pose_topic')),
            ('/grid_map/depth', LaunchConfiguration('depth_topic'))
        ],
        # 节点参数配置
        parameters=[{
            # 规划状态机参数
            'fsm/flight_type': LaunchConfiguration('flight_type'),
            'fsm/thresh_replan': 1.5,
            'fsm/thresh_no_replan': 2.0,
            'fsm/planning_horizon': LaunchConfiguration('planning_horizon'),
            'fsm/planning_horizen_time': 3.0,
            'fsm/emergency_time_': 1.0,
            'fsm/w_adjust_': 1.0,
            
            # 航点参数
            'fsm/waypoint_num': LaunchConfiguration('point_num'),
            'fsm/waypoint0_x': LaunchConfiguration('point0_x'),
            'fsm/waypoint0_y': LaunchConfiguration('point0_y'),
            'fsm/waypoint0_z': LaunchConfiguration('point0_z'),
            'fsm/waypoint1_x': LaunchConfiguration('point1_x'),
            'fsm/waypoint1_y': LaunchConfiguration('point1_y'),
            'fsm/waypoint1_z': LaunchConfiguration('point1_z'),
            'fsm/waypoint2_x': LaunchConfiguration('point2_x'),
            'fsm/waypoint2_y': LaunchConfiguration('point2_y'),
            'fsm/waypoint2_z': LaunchConfiguration('point2_z'),
            'fsm/waypoint3_x': LaunchConfiguration('point3_x'),
            'fsm/waypoint3_y': LaunchConfiguration('point3_y'),
            'fsm/waypoint3_z': LaunchConfiguration('point3_z'),
            'fsm/waypoint4_x': LaunchConfiguration('point4_x'),
            'fsm/waypoint4_y': LaunchConfiguration('point4_y'),
            'fsm/waypoint4_z': LaunchConfiguration('point4_z'),
            
            # 网格地图参数
            'grid_map/resolution': 0.1,
            'grid_map/map_size_x': LaunchConfiguration('map_size_x_'),
            'grid_map/map_size_y': LaunchConfiguration('map_size_y_'),
            'grid_map/map_size_z': LaunchConfiguration('map_size_z_'),
            'grid_map/local_update_range_x': 8.0,
            'grid_map/local_update_range_y': 8.0,
            'grid_map/local_update_range_z': 1.5,
            'grid_map/obstacles_inflation': 0.5,
            'grid_map/local_map_margin': 2,
            'grid_map/ground_height': -0.01,
            
            # 相机参数
            'grid_map/cx': LaunchConfiguration('cx'),
            'grid_map/cy': LaunchConfiguration('cy'),
            'grid_map/fx': LaunchConfiguration('fx'),
            'grid_map/fy': LaunchConfiguration('fy'),
            
            # 深度滤波参数
            'grid_map/use_depth_filter': True,
            'grid_map/depth_filter_tolerance': 0.15,
            'grid_map/depth_filter_maxdist': 5.0,
            'grid_map/depth_filter_mindist': 0.2,
            'grid_map/depth_filter_margin': 1,
            'grid_map/k_depth_scaling_factor': 1000.0,
            'grid_map/skip_pixel': 2,
            
            # 局部融合参数
            'grid_map/p_hit': 0.65,
            'grid_map/p_miss': 0.35,
            'grid_map/p_min': 0.12,
            'grid_map/p_max': 0.90,
            'grid_map/p_occ': 0.80,
            'grid_map/min_ray_length': 0.1,
            'grid_map/max_ray_length': 4.0,
            
            # 其他地图参数
            'grid_map/virtual_ceil_height': 2.5,
            'grid_map/visualization_truncate_height': 1.6,
            'grid_map/show_occ_time': False,
            'grid_map/pose_type': 2,
            'grid_map/map_type': 1,
            'grid_map/frame_id': 'map',
            
            # 规划管理器参数
            'manager/max_vel': LaunchConfiguration('max_vel'),
            'manager/max_acc': LaunchConfiguration('max_acc'),
            'manager/max_jerk': 4.0,
            'manager/control_points_distance': 0.6,
            'manager/feasibility_tolerance': 0.05,
            'manager/planning_horizon': LaunchConfiguration('planning_horizon'),
            
            # 轨迹优化参数
            'optimization/lambda_smooth': 1.0,
            'optimization/lambda_collision': 0.8,
            'optimization/lambda_feasibility': 0.1,
            'optimization/lambda_fitness': 1.0,
            'optimization/dist0': 1.0,
            'optimization/max_vel': LaunchConfiguration('max_vel'),
            'optimization/max_acc': LaunchConfiguration('max_acc'),
            
            # B样条参数
            'bspline/limit_vel': LaunchConfiguration('max_vel'),
            'bspline/limit_acc': LaunchConfiguration('max_acc'),
            'bspline/limit_ratio': 1.1,
            
            # # MPC参数
            # 'MPC/v_max': 1.8,
            # 'MPC/w_max': 1.2,
            # 'MPC/omega0': 1.0,
            # 'MPC/omega1': 0.1
        }]
    )
    
    # 组合所有启动项
    return LaunchDescription(args + [ego_planner_node])
