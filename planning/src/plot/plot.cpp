#include"plot.h"

namespace plt = matplotlibcpp;
namespace carla_pnc {
Plot::Plot() {}
Plot::~Plot() {}

void Plot::PlotRoutePath(const std::vector<path_point>& path_points,
                         const std::string& color) {
  std::vector<double> x, y;
  for (const auto& point : path_points) {
    x.emplace_back(point.x);
    y.emplace_back(point.y);
  }
  // plt::plot(x, y, color);
  plt::scatter(x, y);  // 离散点
  // plt::show();
}

void Plot::PlotRefPath(const std::vector<path_point>& path_points,
                       const std::string& color) {
  std::vector<double> x, y;
  for (const auto& point : path_points) {
    x.emplace_back(point.x);
    y.emplace_back(point.y);
  }
  plt::named_plot("ref line", x, y, color);
  plt::xlabel("x(m)");
  plt::ylabel("y(m)");
  plt::legend();
  plt::show();
}

void Plot::PlotRawCurvature(const std::vector<path_point>& path_points,
                            const std::string& color) {
  std::vector<double> x, curvature;
  for (const auto& point : path_points) {
    x.emplace_back(point.x);
    curvature.emplace_back(point.cur);
  }
  plt::named_plot("raw line", x, curvature, color);
  plt::legend();
}

void Plot::PlotRefCurvature(const std::vector<path_point>& path_points,
                         const std::string& color) {
  std::vector<double> x, curvature;
  for (const auto& point : path_points) {
    x.emplace_back(point.x);
    curvature.emplace_back(point.cur);
  }
  plt::named_plot("ref line", x, curvature, color);
  plt::xlabel("x(m)");
  plt::ylabel("curvature(1/m)");
  plt::legend();
  plt::show();
}

}  // namespace carla_pnc
