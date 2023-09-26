#include"plot.h"

namespace plt = matplotlibcpp;
namespace carla_pnc {
Plot::Plot() {}
Plot::~Plot() {}

void Plot::PlotLocalPath(const std::vector<path_point>& path_points,
                         const std::string& color) {
  std::vector<double> x, y;
  for (const auto& point : path_points) {
    x.emplace_back(point.x);
    y.emplace_back(point.y);
  }
  plt::plot(x, y, color);
  // plt::show();
}

void Plot::PlotRefPath(const std::vector<path_point>& path_points,
                       const std::string& color) {
  std::vector<double> x, y;
  for (const auto& point : path_points) {
    x.emplace_back(point.x);
    y.emplace_back(point.y);
  }
  plt::plot(x, y, color);
  // plt::title("1");
  plt::show();
}

void Plot::PlotCurvature(const std::vector<path_point>& path_points) {
  std::vector<double> x, curvature;
  for (const auto& point : path_points) {
    x.emplace_back(point.x);
    curvature.emplace_back(point.cur);
  }

  plt::plot(x, curvature);
  plt::title("curvature");
  plt::show();
}

}  // namespace carla_pnc
