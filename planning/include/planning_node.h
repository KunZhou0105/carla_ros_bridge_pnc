/**
 * @file planning_node.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef PLANNING_INCLUDE_PLANNING_NODE_H_
#define PLANNING_INCLUDE_PLANNING_NODE_H_
#include <iostream>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

#include <waypoint_msgs/Waypoint.h>
#include <waypoint_msgs/WaypointArray.h>
#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include "collision_detection/collision_detection.h"
#include "reference_line/reference_line.h"
#include "lattice_planner/lattice_planner.h"
#include "em_planner/em_planner.h"
#include "reference_line/cubic_spline.hpp"
#include "point_types.h"
#include "common.h"
#include "/home/zk/carla_ros_bridge_pnc/planning/src/plot/plot.h"

namespace carla_pnc {
class PlanningNode {
 public:
  PlanningNode();
  void MainLoop();
  // 创建路径
  void CreatePath();
  // 仿真测试
  void TestProc();
  // 参考线平滑test
  void RefSmoothTest();

 protected:
  bool bo_test_;                        // 仿真测试开关
  Plot tcl_plot;                        // 画图类
  std::vector<path_point> global_path;  // 全局路径
  car_state cur_pose;                   // 车辆当前状态
  bool planner_activate;                // 是否启用planner
  double path_length;                   // 局部规划参考线长度
  double cruise_speed;                  // 设定的巡航车速 m/s
  bool first_loop;                      // 判断算否为第一次循环
  int pre_match_index;                  // 上一周期的匹配点下标
  bool get_odom;                        // 用于判断是否接受到定位信息
  bool car_follow;                      // 用于判断是否跟车

  std::vector<Obstacle> detected_objects;  // 感知到的障碍物

  std::vector<path_point> local_path;  // 从全局路径截取的参考线（平滑前）
  std::vector<path_point> ref_path;    // 平滑后的参考线

  FrenetPath final_path;      // 最优规划轨迹
  FrenetPath pre_final_path;  // 上一周期的规划轨迹

  std::vector<FrenetPath> sample_paths;   // 用于可视化所有采样轨迹
  std::vector<FrenetPath> history_paths;  // 用于可视化历史采样轨迹

  /***********************************Params**************************************/
  std::string role_name;
  std::string planning_method;
  // collision_detection params
  double collision_distance;

  bool use_discrete_smooth;  // 是否使用离散点平滑（Apollo）默认为false
  std::unordered_map<std::string, double> lattice_params;
  std::unordered_map<std::string, double> referline_params;
  std::unordered_map<std::string, double> dp_path_params;
  std::unordered_map<std::string, double> qp_path_params;

  /***********************************Subscriber**************************************/

  ros::Subscriber cur_pose_sub;          // 获取车辆当前状态
  ros::Subscriber global_path_sub;       // 获取carla规划后的全局路径
  ros::Subscriber cruise_speed_sub;      // 获取巡航车速
  ros::Subscriber imu_sub;               // 获取imu信息
  ros::Subscriber detected_objects_sub;  // 获取障碍物信息

  /***********************************Publisher**************************************/

  ros::Publisher local_waypoints_pub;  // 发布给控制器的局部路径点信息
  ros::Publisher ref_path_pub;         // 参考线可视化
  ros::Publisher sample_paths_pub;     // 发布采样轨迹，用于rviz可视化
  ros::Publisher final_path_pub;       // 发布最优轨迹，用于rviz可视化
  ros::Publisher history_paths_pub;    // 发布历史参考路径，用于rviz可视化

  ros::Publisher speed_marker_pub;  // 发布目标速度，用于rviz可视化
  /***********************************callback**************************************/

  void callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg);

  void callbackCruiseSpeed(const std_msgs::Float32::ConstPtr &msg);

  void callbackGlobalPath(const nav_msgs::Path::ConstPtr &msg);

  void callbackIMU(const sensor_msgs::Imu::ConstPtr &msg);

  void callbackDetectedObjects(const derived_object_msgs::ObjectArray::ConstPtr &msg);

  /*********************************visualization****************************************/
  void ref_path_visualization(const std::vector<path_point> &ref_path);

  void final_path_visualization(const FrenetPath &final_path);

  void sample_paths_visualization(const std::vector<FrenetPath> &sample_paths);

  void history_path_visualization(const std::vector<FrenetPath> &histroy_paths);

  void object_speed_visualization(const std::vector<Obstacle> &detected_objects);
};

}  // namespace carla_pnc
#endif  // PLANNING_INCLUDE_PLANNING_NODE_H_
