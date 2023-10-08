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

  // route
  void PlotRoutePath(const std::vector<path_point>& path_points,
                     const std::string& color);

  // 参考线
  void PlotRefPath(const std::vector<path_point>& path_points,
                   const std::string& color);

  // 平滑前曲率
  void PlotRawCurvature(const std::vector<path_point>& path_points,
                        const std::string& color);
  // 平滑后曲率
  void PlotRefCurvature(const std::vector<path_point>& path_points,
                        const std::string& color);
};
}  // namespace carla_pnc

# endif  // PLANNING_SRC_PLOT_PLOT_H_
