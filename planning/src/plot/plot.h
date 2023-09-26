#ifndef PLANNING_SRC_PLOT_PLOT_H_
#define PLANNING_SRC_PLOT_PLOT_H_

#include <vector>
#include <string>

#include "/home/zk/carla_ros_bridge_pnc/planning/include/point_types.h"
// #include "plannging/include/em_planner/em_planner.h"
#include "matplot/matplotlibcpp.h"
// #include "reference_line/reference_line.h"
namespace plt = matplotlibcpp;

namespace carla_pnc {
class Plot {
 public:
  Plot();
  ~Plot();

  // 局部路径
  void PlotLocalPath(const std::vector<path_point>& path_points,
                     const std::string& color);

  // 参考线
  void PlotRefPath(const std::vector<path_point>& path_points,
                   const std::string& color);

  // 曲率
  void PlotCurvature(const std::vector<path_point>& path_points);
};
}  // namespace carla_pnc

# endif  // PLANNING_SRC_PLOT_PLOT_H_
