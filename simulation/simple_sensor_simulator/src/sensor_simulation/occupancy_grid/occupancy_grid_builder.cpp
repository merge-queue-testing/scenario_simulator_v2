// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include <quaternion_operation/quaternion_operation.h>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_builder.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_traversal.hpp>

namespace simple_sensor_simulator
{
OccupancyGridBuilder::OccupancyGridBuilder(
  double resolution, size_t height, size_t width, int8_t occupied_cost, int8_t invisible_cost)
: resolution(resolution),
  height(height),
  width(width),

  occupied_cost(occupied_cost),
  invisible_cost(invisible_cost),

  occupied_grid_(height * width),
  invisible_grid_(height * width),
  values_(height * width),

  mincols_(height),
  maxcols_(height)
{
}

auto OccupancyGridBuilder::makePoint(double x, double y, double z = 0) const -> PointType
{
  auto p = PointType();
  p.x = x, p.y = y, p.z = z;
  return p;
}

auto OccupancyGridBuilder::transformToGrid(const PointType & world_point) const -> PointType
{
  namespace quat_op = quaternion_operation;
  auto rot = quat_op::getRotationMatrix(quat_op::conjugate(origin_.orientation));
  auto p = Eigen::Vector3d(world_point.x, world_point.y, world_point.z);
  auto q = Eigen::Vector3d(origin_.position.x, origin_.position.y, origin_.position.z);
  p = rot * p - q;
  return makePoint(p(0), p(1), p(2));
}

auto OccupancyGridBuilder::transformToPixel(const PointType & grid_point) const -> PointType
{
  return makePoint(
    (grid_point.x + height * resolution * 0.5) / resolution,
    (grid_point.y + width * resolution * 0.5) / resolution);
}

auto OccupancyGridBuilder::makeOccupiedArea(const PrimitiveType & primitive) const -> PolygonType
{
  auto res = primitive.get2DConvexHull();
  for (auto & p : res) {
    p = transformToGrid(p);
  }
  return res;
}

auto OccupancyGridBuilder::makeInvisibleArea(const PolygonType & occupied) const -> PolygonType
{
  const auto realw = width * resolution / 2;
  const auto realh = height * resolution / 2;

  const auto corners = [&](size_t i) {
    switch (i % 4) {
      default:
        return makePoint(-realw, -realh);  // bottom left
      case 1:
        return makePoint(realw, -realh);  // bottom right
      case 2:
        return makePoint(realw, realh);  // top right
      case 3:
        return makePoint(-realw, realh);  // top left
    }
  };

  const auto projection = [&](const PointType & p, size_t i) {
    switch (i % 4) {
      default:
        return makePoint(-realw, p.y * -realw / p.x);  // left
      case 1:
        return makePoint(p.x * -realh / p.y, -realh);  // bottom
      case 2:
        return makePoint(realw, p.y * realw / p.x);  // right
      case 3:
        return makePoint(p.x * realh / p.y, realh);  // top
    }
  };

  auto res = PolygonType();
  {
    auto [minp, maxp] = [&] {
      auto res = std::minmax_element(
        occupied.begin(), occupied.end(), [&](const PointType & p, const PointType & q) {
          return std::atan2(p.y, p.x) < std::atan2(q.y, q.x);
        });

      auto [minp, maxp] = res;
      if (std::atan2(maxp->y, maxp->x) - std::atan2(minp->y, minp->x) > M_PI) {
        res = std::minmax_element(
          occupied.begin(), occupied.end(), [&](const PointType & p, const PointType & q) {
            auto prad = std::atan2(p.y, p.x);
            auto qrad = std::atan2(q.y, q.x);

            if (prad < 0) prad += 2 * M_PI;
            if (qrad < 0) qrad += 2 * M_PI;

            return prad < qrad;
          });
      }
      return res;
    }();

    double minang = std::atan2(minp->y, minp->x);
    double maxang = std::atan2(maxp->y, maxp->x);
    if (minang > maxang) maxang += 2 * M_PI;

    size_t i = 0;
    for (;; ++i) {
      auto corner = corners(i);
      if (std::atan2(corner.y, corner.x) >= minang) break;
    }

    res.emplace_back(*minp);
    res.emplace_back(projection(*minp, i));

    for (;; ++i) {
      auto corner = corners(i);
      if (std::atan2(corner.y, corner.x) + 2 * M_PI * (i / 4) >= maxang) break;
      res.emplace_back(corner);
    }

    res.emplace_back(projection(*maxp, i));
    res.emplace_back(*maxp);
  }
  return res;
}

auto OccupancyGridBuilder::addPolygon(MarkerGridType & grid, const PolygonType & convex_hull) -> void
{
  mincols_.assign(mincols_.size(), width);
  maxcols_.assign(maxcols_.size(), -1);

  for (size_t i = 0; i < convex_hull.size(); ++i) {
    const auto p = transformToPixel(convex_hull[i]);
    const auto q = transformToPixel(convex_hull[(i + 1) % convex_hull.size()]);
    for (auto [row, col] : GridTraversal(p.x, p.y, q.x, q.y)) {
      if (row >= 0 && row < int32_t(height)) {
        mincols_[row] = std::min(mincols_[row], col);
        maxcols_[row] = std::max(maxcols_[row], col);
      }
    }
  }

  for (size_t row = 0; row < height; ++row) {
    if (auto col = mincols_[row]; col >= 0 && col < int32_t(width)) {
      grid[width * row + col] += 1;
    }
    if (auto col = maxcols_[row]; col >= 0 && col + 1 < int32_t(width)) {
      grid[width * row + col + 1] -= 1;
    }
  }
}

auto OccupancyGridBuilder::add(const PrimitiveType & primitive) -> void
{
  {
    constexpr auto count_max = std::numeric_limits<MarkerCounterType>::max();
    if (primitive_count_++ == count_max) {
      throw std::runtime_error(
        "Grid cannot hold more than " + std::to_string(count_max) + " primitives");
    }
  }

  auto occupied_area = makeOccupiedArea(primitive);

  auto invisible_area = makeInvisibleArea(occupied_area);

  // mark invisible area
  addPolygon(invisible_grid_, invisible_area);

  // mark occupied area
  addPolygon(occupied_grid_, occupied_area);
}

auto OccupancyGridBuilder::build() -> void
{
  // Imos Method
  // https://imoz.jp/algorithms/imos_method.html (Japanese)

  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col + 1 < width; ++col) {
      invisible_grid_[row * width + col + 1] += invisible_grid_[row * width + col];
    }
  }

  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col + 1 < width; ++col) {
      occupied_grid_[row * width + col + 1] += occupied_grid_[row * width + col];
    }
  }

  for (size_t i = 0; i < height * width; ++i) {
    values_[i] = occupied_grid_[i] ? occupied_cost : invisible_grid_[i] ? invisible_cost : 0;
  }
}

auto OccupancyGridBuilder::get() const -> const OccupancyGridType & { return values_; }

auto OccupancyGridBuilder::reset(const PoseType & origin) -> void
{
  origin_ = origin;
  primitive_count_ = 0;
  invisible_grid_.assign(invisible_grid_.size(), 0);
  occupied_grid_.assign(occupied_grid_.size(), 0);
}

}  // namespace simple_sensor_simulator
