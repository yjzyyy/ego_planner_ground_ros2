#include "bspline_opt/uniform_bspline.h"
#include "mpc_controller/MPC.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "std_msgs/msg/u_int8.hpp"    
#include "std_msgs/msg/u_int8_multi_array.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/transform_listener.h"    
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
//#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/msg/empty.hpp" 
#include "std_msgs/msg/bool.hpp" 
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"   
#include <chrono>     

#define PI 3.1415926
#define yaw_error_max 90.0/180*PI
#define N 15


rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_pub;


//quadrotor_msgs::PositionCommand cmd;
geometry_msgs::msg::TwistStamped cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool is_orientation_init = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_,time_s,time_e;
int traj_id_;

Eigen::Vector3d odom_pos_,odom_vel_;
Eigen::Quaterniond odom_orient_;

MPC_controller mpc_controller;
double roll,pitch,yaw;
geometry_msgs::msg::PoseStamped pose_cur;
tf2::Quaternion quat;
std_msgs::msg::UInt8 is_adjust_pose;
std_msgs::msg::UInt8 dir;

enum DIRECTION {POSITIVE=0,NEGATIVE=1};

// yaw control
double t_step;

std_msgs::msg::UInt8 stop_command;

////time record
rclcpp::Time start_clock,end_clock;
// double duration;

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = msg->start_time;

  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  //ROS_INFO("Receive b-spline trajectory!");

  receive_traj_ = true;

}

void poseCallback(geometry_msgs::msg::PoseStamped::ConstPtr msg)
{
    pose_cur = *msg;
    tf2::convert(msg->pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

void adjust_yaw_Callback(std_msgs::msg::UInt8::ConstPtr msg)
{
    is_adjust_pose = *msg;
}

void dirCallback(const std_msgs::msg::UInt8::ConstPtr& msg)
{
    dir = *msg;
}

void MPC_calculate(double &t_cur)
{
    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    Eigen::MatrixXd u_k;
    Eigen::Vector3d pos_r,pos_r_1,pos_final,v_r_1,v_r_2,X_k;
    Eigen::Vector2d u_r;
    Eigen::Vector3d x_r,x_r_1,x_r_2;
    double v_linear_1,w;
    double t_k,t_k_1;

    //ROS_INFO("Run to here!");

    //Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
    //Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
    Eigen::Vector3d vel_start = traj_[1].evaluateDeBoor(t_cur);
    double yaw_start = atan2(vel_start(1),vel_start(0));
    bool is_orientation_adjust=false;
    double orientation_adjust=0;
    pos_final = traj_[0].evaluateDeBoor(traj_duration_);

//    if(abs(yaw-yaw_start)>yaw_error_max&&is_orientation_init==false)
//    {
//        ROS_INFO("current yaw : %5.3f , start yaw : %5.3f",yaw,yaw_start);
//        if(abs(yaw-yaw_start)>PI)
//        {
//            cmd.twist.linear.x=0;
//            cmd.twist.angular.z = -(yaw_start-yaw)/abs(yaw-yaw_start)*PI/10;
//        }
//        else
//        {
//            cmd.twist.linear.x=0;
//            cmd.twist.angular.z = (yaw_start-yaw)/abs(yaw-yaw_start)*PI/10;
//        }
//        vel_cmd_pub.publish(cmd);
//        start_time_ = ros::Time::now();
//        //cout<<"current yaw : "<<yaw*180/PI<<endl;
//        //cout<<"target yaw : "<<yaw_start*180/PI<<endl;
//        //cout<<"current w : "<<web_cmd_vel.angular.z<<endl;
//    }
//    else
//    {
        is_orientation_init=true;
        for(int i=0;i<N;i++)
        {

            t_k = t_cur+i*t_step;
            t_k_1 = t_cur+(i+1)*t_step;

            pos_r = traj_[0].evaluateDeBoor(t_k);
            pos_r_1 = traj_[0].evaluateDeBoor(t_k_1);

            x_r(0) = pos_r(0);
            x_r(1) = pos_r(1);

            v_r_1 = traj_[1].evaluateDeBoor(t_k);
            v_r_2 = traj_[1].evaluateDeBoor(t_k_1);
            v_r_1(2)=0;
            v_r_2(2)=0;
            v_linear_1 = v_r_1.norm();
            if((t_k-traj_duration_)>=0)
            {
                x_r(2) = atan2((pos_r-pos_final)(1),(pos_r-pos_final)(0));
            }
            else
            {
                //x_r(2) = atan2((pos_r_1-pos_r)(1),(pos_r_1-pos_r)(0));
                x_r(2) = atan2(v_r_1(1),v_r_1(0));
            }



            double yaw1 = atan2(v_r_1(1),v_r_1(0));
            double yaw2 = atan2(v_r_2(1),v_r_2(0));

            if(abs(yaw2-yaw1)>PI)
            {
                //ROS_WARN("orientation suddenly change !");
                //cout<<"current index : "<<i+1<<endl;
                //cout<<"yaw 1 : "<<yaw1<<endl;
                //cout<<"yaw 2 : "<<yaw2<<endl;
                is_orientation_adjust = true;
                if((yaw2-yaw1)<0)
                {
                    orientation_adjust = 2*PI;
                    w = (2*PI+(yaw2-yaw1))/t_step;
                }
                else
                {
                    w = ((yaw2-yaw1)-2*PI)/t_step;
                    orientation_adjust = -2*PI;
                }
            }
            else
            {
                w = (yaw2-yaw1)/t_step;
            }

            if(is_orientation_adjust==true)
            {
                //cout<<"orientation before adjust : "<< x_r(2)<<endl;
                x_r(2) +=orientation_adjust;
                //cout<<"orientation after adjust : "<< x_r(2)<<endl;
            }

            u_r(0) = v_linear_1;
            u_r(1) = w;
//                if(t_c>(tp-5*t_step))
//                {
//                    cout<<"Ur "<<i+1<<" : "<<endl<<u_r<<endl;
//                    cout<<"Xr "<<i+1<<" : "<<endl<<x_r<<endl;
//                }
            X_r.push_back(x_r);
            U_r.push_back(u_r);
        }

        //X_k(0) = odom_map.pose.pose.position.x - my_map.info.origin.position.x;
        //X_k(1) = odom_map.pose.pose.position.y - my_map.info.origin.position.y;
        X_k(0) = odom_pos_(0);
        X_k(1) = odom_pos_(1);
        if(yaw/X_r[0](2)<0&&abs(yaw)>(PI*5/6))
        {
            if(yaw<0)
            {
                X_k(2) = yaw + 2*PI;
            }
            else
            {
                X_k(2) = yaw - 2*PI;
            }
        }
        else
        {
            X_k(2) = yaw;
        }
        // cout<<"xr  : "<<X_r[0]<<endl;
        // cout<<"xk  : "<<X_k<<endl;

        //ROS_INFO("Run to here!");
        u_k = mpc_controller.MPC_Solve_qp(X_k,X_r,U_r,N);


//            cout<<"Xk "<<" : "<<endl<<X_k<<endl;
        if(dir.data == NEGATIVE)
        {
            cmd.twist.linear.x = -u_k.col(0)(0);
        }
        else
        {
            cmd.twist.linear.x = u_k.col(0)(0);
        }

        cmd.twist.angular.z = u_k.col(0)(1);
       static int conut1 = 0;
       conut1+=1;
       if(conut1%20==0)
       {
           RCLCPP_WARN(rclcpp::get_logger("traj_server"),"U r :");
           for(int i=0;i<U_r.size();i++)
           {
               cout<<"vel ref :"<<U_r[i](0)<<","<<"w ref : "<<U_r[i](1);
               cout<<endl;
           }
           cout<<endl;
           RCLCPP_WARN(rclcpp::get_logger("traj_server"),"U k :");
           for(int i=0;i<u_k.cols();i++)
           {
               cout<<"vel optimal :"<<u_k.col(i)(0)<<","<<"w optimal : "<<u_k.col(i)(1);
               cout<<endl;
           }
           cout<<endl;
           cout<<"current vel : : "<<u_k.col(0)(0) <<"m/s"<<endl;
           cout<<"current w : "<<u_k.col(0)(1)<<"rad/s"<<endl;
           conut1=0;
       }


        vel_cmd_pub->publish(cmd);
//        control_times+=1;
//        //cout<<"control_times : "<<control_times<<endl;
//        //cout<<"current t : "<<t_c<<endl;
//        if(t_c>tp)
//        {
//            is_trajectory_trace=false;
//            is_orientation_init=false;
//            web_cmd_vel.angular.z = 0;
//            web_cmd_vel.linear.x = 0;
//            web_cmd_pub.publish(web_cmd_vel);
//            control_times=0;
//        }
        //ros::Time t_end = ros::Time::now();
        //ROS_INFO("control total time : %5.3f ms",(t_end-t_start).toSec()*1000);
   // }

}

void stopCallback(std_msgs::msg::UInt8::ConstPtr msg)
{
    stop_command = *msg;
}

void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    tf2::convert(msg->pose.pose.orientation,quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(dir.data==NEGATIVE)
    {
        if(yaw>0)
        {
            yaw -= PI;
        }else if(yaw<0)
        {
            yaw += PI;
        }
    }

}

void cmdCallback()
{
    /* no publishing before receive traj_ */
    if (stop_command.data==1)
    {
        cmd.twist.angular.z = 0;
        cmd.twist.linear.x = 0;
        vel_cmd_pub->publish(cmd);
        std::cout << "[Traj server]: emergency stop !" << std::endl;
        return;
    }

    if (!receive_traj_)
        return;
    //ROS_WARN("Run here !");
    //统一时间基准
    rclcpp::Clock clock(RCL_ROS_TIME);
    rclcpp::Time time_s = clock.now();
    double t_cur = (time_s - start_time_).seconds();

//    Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
//    Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
//    double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));

    static rclcpp::Time time_last = clock.now();

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        //ROS_INFO("MPC_Calculate!");
        start_clock = clock.now();
        MPC_calculate(t_cur);
        end_clock = clock.now();
        rclcpp::Duration duration = (end_clock - start_clock);
        

        // duration = (end_clock - start_clock) / CLOCKS_PER_SEC *1000;
        //ROS_INFO("Control times : %f ms",duration);
    }
    else if (t_cur >= traj_duration_)
    {
        cmd.twist.angular.z = 0;
        cmd.twist.linear.x = 0;
        vel_cmd_pub->publish(cmd);
        is_orientation_init=false;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_s;

    vel_cmd_pub->publish(cmd);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  std::string cmd_topic,pose_topic;
  node->declare_parameter<std::string>("/ego_planner_node/fsm/pose_topic", "/cmd_pos");
  node->declare_parameter<std::string>("/ego_planner_node/fsm/vel_topic", "/cmd_vel");
  
  node->get_parameter("/ego_planner_node/fsm/pose_topic", pose_topic);
  node->get_parameter("/ego_planner_node/fsm/vel_topic", cmd_topic);



//   ros::Subscriber bspline_sub = node.subscribe("/planning/bspline", 10, bsplineCallback);
//   ros::Subscriber pose_sub = node.subscribe(pose_topic, 10, poseCallback);
//   ros::Subscriber odom_sub = node.subscribe("/state_estimation", 10, odometryCallback);
//   ros::Subscriber stop_sub = node.subscribe("/emergency_stop",10,stopCallback);
//   ros::Subscriber adjust_yaw_sub = node.subscribe("/is_adjust_yaw",10,adjust_yaw_Callback);
//   ros::Subscriber command_sub = node.subscribe("/direction",10,dirCallback);


auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "/planning/bspline", 10,
      std::bind(&bsplineCallback, std::placeholders::_1));

  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      std::bind(&poseCallback, std::placeholders::_1));

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10,
      std::bind(&odometryCallback, std::placeholders::_1));

  auto stop_sub = node->create_subscription<std_msgs::msg::UInt8>(
      "/emergency_stop", 10,
      std::bind(&stopCallback, std::placeholders::_1));

  auto adjust_yaw_sub = node->create_subscription<std_msgs::msg::UInt8>(
      "/is_adjust_yaw", 10,
      std::bind(&adjust_yaw_Callback, std::placeholders::_1));

  auto command_sub = node->create_subscription<std_msgs::msg::UInt8>(
      "/direction", 10,
      std::bind(&dirCallback, std::placeholders::_1));



  mpc_controller.MPC_init(node);
  vel_cmd_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 50);
  stop_command.data = false;
  dir.data = POSITIVE;
  t_step = 0.03;


  auto cmd_timer = node->create_wall_timer(
      std::chrono::duration<double>(0.03),
      std::bind(&cmdCallback));

  // ROS 2睡眠
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  // ROS 2 spin
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}