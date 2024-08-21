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

#include <cmath>
#include <geometry/intersection/intersection.hpp>
#include <geometry/polygon/line_segment.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/transform.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/operator.hpp>
#include <iostream>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>

namespace math
{
namespace geometry
{
LineSegment::LineSegment(
  const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Point & end_point)
: start_point(start_point),
  end_point(end_point),
  length(hypot(end_point, start_point)),
  length2D(std::hypot(end_point.x - start_point.x, end_point.y - start_point.y))
{
}

LineSegment::LineSegment(
  const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Vector3 & vec,
  double length)
: start_point(start_point),
  end_point([&]() -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point ret;
    double vec_size = std::hypot(vec.x, vec.y);
    if (vec_size == 0.0) {
      THROW_SIMULATION_ERROR(
        "Invalid vector is specified, while constructing LineSegment. ",
        "The vector should have a non zero length to initialize the line segment correctly. ",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    }
    ret.x = start_point.x + vec.x / vec_size * length;
    ret.y = start_point.y + vec.y / vec_size * length;
    ret.z = start_point.z + vec.z / vec_size * length;
    return ret;
  }()),
  length(hypot(end_point, start_point)),
  length2D(std::hypot(end_point.x - start_point.x, end_point.y - start_point.y))
{
}

LineSegment::~LineSegment() {}

/**
 * @brief Get point on the line segment from s value.
 * @param s Normalized s value in coordinate along line segment.
 * @param denormalize_s If true, s value should be normalized in range [0,1]. If false, s value is not normalized.
 * @return geometry_msgs::msg::Point 
 */
auto LineSegment::getPoint(const double s, const bool denormalize_s) const
  -> geometry_msgs::msg::Point
{
  const double s_normalized = denormalize_s ? s / getLength() : s;
  if (0 <= s_normalized && s_normalized <= 1) {
    return geometry_msgs::build<geometry_msgs::msg::Point>()
      .x(start_point.x + (end_point.x - start_point.x) * s_normalized)
      .y(start_point.y + (end_point.y - start_point.y) * s_normalized)
      .z(start_point.z + (end_point.z - start_point.z) * s_normalized);
  }
  if (denormalize_s) {
    THROW_SIMULATION_ERROR(
      "Invalid S value is specified, while getting point on a line segment.",
      "The range of s_normalized value should be in range [0,", getLength(), "].",
      "But, your values are = ", s, " and length = ", getLength(),
      " This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  } else {
    THROW_SIMULATION_ERROR(
      "Invalid S value is specified, while getting point on a line segment.",
      "The range of s_normalized value should be in range [0,1].", "But, your values are = ", s,
      " and length = ", getLength(),
      " This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  }
}

/**
 * @brief Get pose on the line segment from s value. Orientation of thee return value was calculated from the vector of the line segment.
 * @param s Normalized s value in coordinate along line segment.
 * @param denormalize_s If true, s value should be normalized in range [0,1]. If false, s value is not normalized.
 * @param fill_pitch If true, the pitch value of the orientation is filled. If false, the pitch value of the orientation is 0.
 *        This parameter is introduced for backward-compatibility.
 * @return geometry_msgs::msg::Pose 
 */
auto LineSegment::getPose(const double s, const bool denormalize_s, const bool fill_pitch) const
  -> geometry_msgs::msg::Pose
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(getPoint(s, denormalize_s))
    .orientation([this, fill_pitch]() -> geometry_msgs::msg::Quaternion {
      const auto tangent_vec = getVector();
      return math::geometry::convertEulerAngleToQuaternion(
        geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(0.0)
          .y(
            fill_pitch ? std::atan2(-tangent_vec.z, std::hypot(tangent_vec.x, tangent_vec.y)) : 0.0)
          .z(std::atan2(tangent_vec.y, tangent_vec.x)));
    }());
}

/**
 * @brief Checking the intersection with 1 line segment and 1 point in 2D (x,y) coordinate. Ignore z axis.
 * @param point point you want to check intersection.
 * @return true Intersection detected.
 * @return false Intersection not detected.
 */
auto LineSegment::isIntersect2D(const geometry_msgs::msg::Point & point) const -> bool
{
  const bool x_outside = ((point.x - start_point.x) * (end_point.x - point.x) < 0.0);
  const bool y_outside = ((point.y - start_point.y) * (end_point.y - point.y) < 0.0);
  if (x_outside || y_outside) {
    return false;
  }
  const double cross_product = (point.y - start_point.y) * (end_point.x - start_point.x) -
                               (point.x - start_point.x) * (end_point.y - start_point.y);
  constexpr double tolerance = std::numeric_limits<double>::epsilon();
  return std::abs(cross_product) <= get2DLength() * tolerance;
}

auto LineSegment::getSValue(
  const geometry_msgs::msg::Pose & pose, double threshold_distance, bool denormalize_s) const
  -> std::optional<double>
{
  return getIntersection2DSValue(
    LineSegment(
      math::geometry::transformPoint(
        pose, geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(threshold_distance).z(0)),
      math::geometry::transformPoint(
        pose, geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-threshold_distance).z(0))),
    denormalize_s);
}

/**
 * @brief Checking the intersection with 2 line segments in 2D (x,y) coordinate. Ignore z axis.
 * @param line line segments you want to check intersection.
 * @return true Intersection detected.
 * @return false Intersection not detected.
 */
auto LineSegment::isIntersect2D(const LineSegment & line) const -> bool
{
  return math::geometry::isIntersect2D(*this, line);
}

/**
 * @brief Find intersection point of 1 line segment and 1 point.
 * @param point point of you want to find intersection.
 * @return std::optional<double> 
 */
auto LineSegment::getIntersection2DSValue(
  const geometry_msgs::msg::Point & point, const bool denormalize_s) const -> std::optional<double>
{
  if (isIntersect2D(point)) {
    const double distance = std::hypot(point.x - start_point.x, point.y - start_point.y);
    return denormalize_s ? std::make_optional(distance)
                         : std::make_optional(distance / get2DLength());
  } else {
    return std::nullopt;
  }
}

/**
 * @brief Get S value of the intersection point of two line segment.
 * @param line The line segment you want to check intersection.
 * @param denormalize_s If true, s value should be normalized in range [0,1]. If false, s value is not normalized.
 * @return std::optional<double> 
 */
auto LineSegment::getIntersection2DSValue(const LineSegment & line, const bool denormalize_s) const
  -> std::optional<double>
{
  /// @note Hard coded parameter, this parameter describes the tolerance of the range of s value (-s_tolerance ~ 1.0 + s_tolerance)
  constexpr double s_tolerance = 1e-10;
  const auto get_s_normalized = [this](const auto & line) -> std::optional<double> {
    if (!isIntersect2D(line)) {
      return std::optional<double>();
    }
    const double det = (start_point.x - end_point.x) * (line.end_point.y - line.start_point.y) -
                       (line.end_point.x - line.start_point.x) * (start_point.y - end_point.y);
    const double s =
      1 - ((line.end_point.y - line.start_point.y) * (line.end_point.x - end_point.x) +
           (line.start_point.x - line.end_point.x) * (line.end_point.y - end_point.y)) /
            det;
    if (std::isnan(s)) {
      THROW_SIMULATION_ERROR(
        "One line segment is on top of the other. So determinant is zero.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    }
    return (-s_tolerance <= s && s <= 1 + s_tolerance)
             ? std::optional<double>(std::clamp(s, 0.0, 1.0))
             : std::optional<double>();
  };
  return denormalize_s ? denormalize(get_s_normalized(line)) : get_s_normalized(line);
}

/**
 * @brief Find intersection point of two line segments.
 * @param line Line segment of you want to find intersection.
 * @return std::optional<geometry_msgs::msg::Point> Intersection point, if the value is std::nullopt, lines have no intersection point. 
 */
auto LineSegment::getIntersection2D(const LineSegment & line) const
  -> std::optional<geometry_msgs::msg::Point>
{
  const auto s = getIntersection2DSValue(line, false);
  return s ? geometry_msgs::build<geometry_msgs::msg::Point>()
               .x(s.value() * start_point.x + (1.0 - s.value()) * end_point.x)
               .y(s.value() * start_point.y + (1.0 - s.value()) * end_point.y)
               .z(s.value() * start_point.z + (1.0 - s.value()) * end_point.z)
           : std::optional<geometry_msgs::msg::Point>();
}

auto LineSegment::getVector() const -> geometry_msgs::msg::Vector3
{
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(end_point.x - start_point.x)
    .y(end_point.y - start_point.y)
    .z(end_point.z - start_point.z);
}

/**
 * @brief Get normal vector of the line segment.
 * @return geometry_msgs::msg::Vector3 
 */
auto LineSegment::getNormalVector() const -> geometry_msgs::msg::Vector3
{
  geometry_msgs::msg::Vector3 tangent_vec = getVector();
  double theta = M_PI / 2.0;
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(tangent_vec.x * std::cos(theta) - tangent_vec.y * std::sin(theta))
    .y(tangent_vec.x * std::sin(theta) + tangent_vec.y * std::cos(theta))
    .z(0.0);
}

auto LineSegment::get2DVector() const -> geometry_msgs::msg::Vector3
{
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(end_point.x - start_point.x)
    .y(end_point.y - start_point.y)
    .z(0.0);
}

auto LineSegment::get2DLength() const -> double { return length2D; }

auto LineSegment::getLength() const -> double { return length; }

auto LineSegment::getSlope() const -> double
{
  return (end_point.y - start_point.y) / (end_point.x - start_point.x);
}

/**
 * @brief Get squared distance (Square of euclidean distance) between specified 3D point and specified 3D point on line segment in 2D. (x,y)
 * @param point Specified 3D point
 * @param S value of specified 3D point in coordinate along line segment.
 * @param denormalize_s If true, the s value is denormalized. If false, the s value should be normalized in range [0,1].
 * @return double 
 */
auto LineSegment::getSquaredDistanceIn2D(
  const geometry_msgs::msg::Point & point, const double s, const bool denormalize_s) const -> double
{
  const auto point_on_line = getPoint(s, denormalize_s);
  return std::pow(point.x - point_on_line.x, 2) + std::pow(point.y - point_on_line.y, 2);
}

/**
 * @brief Get 3D vector from specified 3D point to specified 3D point on line segment.
 * @param point Specified 3D point
 * @param s S value of specified 3D point in coordinate along line segment.
 * @param denormalize_s If true, the s value is denormalized. If false, the s value should be normalized in range [0,1].
 * @return geometry_msgs::msg::Vector3 
 */
auto LineSegment::getSquaredDistanceVector(
  const geometry_msgs::msg::Point & point, const double s, const bool denormalize_s) const
  -> geometry_msgs::msg::Vector3
{
  const auto point_on_line = getPoint(s, denormalize_s);
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(point.x - point_on_line.x)
    .y(point.y - point_on_line.y)
    .z(point.z - point_on_line.z);
}

/**
 * @brief Denormalize s value in coordinate along line segment.
 * @param s Normalized s value in coordinate along line segment.
 * @param throw_error_on_out_of_range If true, throw error when the normalized s value is not in range [0,1].
 * If false, return std::nullopt;
 * @return std::optional<double> Denormalize s value. If the `throw_error_on_out_of_range = true` and the normalized s value is not in range [0,1] it is std::nullopt;
 */
auto LineSegment::denormalize(
  const std::optional<double> & s, const bool throw_error_on_out_of_range) const
  -> std::optional<double>
{
  if (!throw_error_on_out_of_range && s && !(0 <= s.value() && s.value() <= 1)) {
    return std::optional<double>();
  }
  return s ? denormalize(s.value()) : std::optional<double>();
}

/**
 * @brief Denormalize s value in coordinate along line segment.
 * @param s Normalized s value in coordinate along line segment.
 * @return double Denormalized s value.
 */
auto LineSegment::denormalize(const double s) const -> double
{
  if (0 <= s && s <= 1) {
    return s * getLength();
  }
  THROW_SIMULATION_ERROR(
    "Invalid normalized s value, s = ", s, ", S value should be in range [0,1].",
    "This message is not originally intended to be displayed, if you see it, please "
    "contact the developer of traffic_simulator.");
}

LineSegment & LineSegment::operator=(const LineSegment &) { return *this; }

/**
 * @brief Get the line segments from points. 
 * @param points Points you want to build line segments.
 * @param close_start_end If true, returned line segments are connected their start and end. 
 * @return std::vector<LineSegment> 
 */
auto getLineSegments(
  const std::vector<geometry_msgs::msg::Point> & points, const bool close_start_end)
  -> std::vector<LineSegment>
{
  if (points.size() <= 1) {
    return {};
  } else {
    std::vector<LineSegment> seg;
    for (size_t i = 0; i < points.size() - 1; i++) {
      seg.emplace_back(LineSegment(points[i], points[i + 1]));
    }
    /// @note If true, the end point(points[points.size() - 1]) and start point(points[0]) was connected.
    if (close_start_end) {
      seg.emplace_back(LineSegment(points[points.size() - 1], points[0]));
    }
    return seg;
  }
}

/**
 * @brief Checks if the given point lies within the bounding box of the line segment.
 * @param point Points you want to test.
 * @return 
 * - `true` if the points is within the bounds.
 * - `false` otherwise.
 */
auto LineSegment::isInBounds2D(const geometry_msgs::msg::Point & point) const -> bool
{
  return (
    point.x >= std::min(start_point.x, end_point.x) &&
    point.x <= std::max(start_point.x, end_point.x) &&
    point.y >= std::min(start_point.y, end_point.y) &&
    point.y <= std::max(start_point.y, end_point.y));
}

/**
 * @brief Determines the relative position of a point with respect to the line segment.
 * 
 * This method computes the relative position of a given point with respect to the line segment defined by 
 * `start_point` and `end_point` using a cross product. The result indicates on which side of the line 
 * segment the point lies.
 * 
 * @param point The point to be evaluated.
 * @return 
 *  - `1` if the point is to the left of the line segment (when moving from `start_point` to `end_point`).
 *  - `-1` if the point is to the right of the line segment.
 *  - `0` if the point is collinear with the line segment (i.e., lies exactly on the line defined by 
 *    `start_point` and `end_point`).
 */
auto LineSegment::relativePointPosition2D(const geometry_msgs::msg::Point & point) const -> int
{
  constexpr double tolerance = std::numeric_limits<double>::epsilon();
  const double determinant = (end_point.y - start_point.y) * (point.x - end_point.x) -
                             (end_point.x - start_point.x) * (point.y - end_point.y);
  return static_cast<int>(determinant > tolerance) - static_cast<int>(determinant < -tolerance);
}

}  // namespace geometry
}  // namespace math
