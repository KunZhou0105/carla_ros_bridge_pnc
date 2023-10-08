/**
 * @file reference_line.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-22
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_H_
#define PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_H_

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"
#include "point_types.h"
#include "reference_line/cubic_spline.hpp"
#include "common.h"

namespace carla_pnc {
class ReferenceLine {
 public:
  double lookahead_dist;
  int match_index;

  ReferenceLine(double lookahead_distance) : lookahead_dist(lookahead_distance), match_index(0) {}

  ReferenceLine(double lookahead_distance,
                std::unordered_map<std::string, double> &referline_params);

  int search_target_index(const double &cur_x, const double &cur_y,
                          const std::vector<path_point> &waypoints,
                          const double &lookahead_distance);

  std::vector<path_point> local_path_truncation(const car_state &cur_pose,
                                                const std::vector<path_point> &global_path,
                                                const int &pre_match_index);

  // cublic Spiline平滑
  std::vector<path_point> smoothing(Spline2D &ref_frenet,
                                    const std::vector<path_point> &local_path);

  // 离散点平滑（Apollo）
  std::vector<path_point> discrete_smooth(const std::vector<path_point> &local_path);
  // 离散点平滑的二次规划求解
  void discrete_points_osqp(std::vector<std::pair<double, double>> &path_point2d);

 private:
  // 离散点平滑平滑相关参数
  double ref_weight_smooth;         // 参考线平滑代价
  double ref_weight_path_length;    // 参考线轨迹长度代价
  double ref_weight_ref_deviation;  // 参考线偏移代价
  // 二次规划几何相似度约束
  double x_lower_bound;
  double x_upper_bound;
  double y_lower_bound;
  double y_upper_bound;
};
}  // namespace carla_pnc
#endif  // PLANNING_INCLUDE_REFERENCE_LINE_REFERENCE_LINE_H_
