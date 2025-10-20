#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
// #include "bspline_opt/uniform_bspline.h"
#include <iostream>
#include <traj_utils/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
class PlanningVisualization
{
public:
  typedef std::shared_ptr<PlanningVisualization> Ptr;
  PlanningVisualization();
  ~PlanningVisualization();
  PlanningVisualization(const rclcpp::Node::SharedPtr &node);

  void displayMarkerList(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, const std::vector<Eigen::Vector3d> &list, double scale,
                         Eigen::Vector4d color, int id, bool show_sphere = true);
  void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
  void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                 const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
  void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
  void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
  void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
  void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
  void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
  void displayArrowList(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);

private:
  rclcpp::Node::SharedPtr node_;
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_point_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_list_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_list_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_list_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_list_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr guide_vector_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_state_pub;
};
} // namespace ego_planner
#endif