/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/angle_utils.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/spline/Spline.h>
#include <units/angle.h>
#include <units/length.h>

#include <vector>

#include "measure_up.h"

namespace field_dimensions {
  // Field max x and y
  constexpr auto fieldMaxY = 315.5975_in;
  constexpr auto fieldMaxX = 651.2225_in;
  constexpr auto fieldMiddleX = fieldMaxX / 2;
}  // namespace field_dimensions

namespace utils {
  // * Note this will only work on points contained in friendly half of field
  /// @brief Reflects the point source over the middle of the field to get equivelent points accross the field
  /// @param source The point to reflect
  /// @return The reflected point accross the middle of the field
  constexpr frc::Translation3d ReflectFieldPoint(const frc::Translation3d source) {
    return frc::Translation3d{source.X(), field_dimensions::fieldMaxY - source.Y(), source.Z()};
  }

  constexpr frc::Translation2d ReflectFieldPoint(const frc::Translation2d source) {
    return frc::Translation2d{source.X(), field_dimensions::fieldMaxY - source.Y()};
  }

  constexpr units::angle::degree_t ReflectAngle(const units::angle::degree_t sourceAngle) {
    return sourceAngle * units::scalar_t{-1};
  }

  template <class T>
  std::vector<T> ReflectFieldPoint(const std::vector<T> source) {
    std::vector<T> retVal;
    retVal.reserve(source.size());
    std::transform(
        source.begin(), source.end(), std::back_inserter(retVal), [](T val) { return ReflectFieldPoint(val); });
    return retVal;
  }

  constexpr frc::Spline<3>::ControlVector ReflectFieldPoint(const frc::Spline<3>::ControlVector source) {
    return frc::Spline<3>::ControlVector{
        .x{source.x}, .y{units::meter_t{field_dimensions::fieldMaxY}.to<double>() - source.y[0], -source.y[1]}};
  }

  frc::Pose2d ReflectFieldPoint(const frc::Pose2d source);

  constexpr units::inch_t ReflectYLine(const units::inch_t source) {
    return field_dimensions::fieldMaxY - source;
  }
}  // namespace utils

namespace field_points {
  namespace blue_alliance {
    // Reference game_piece_positions in Docs directory for conventions
    namespace game_pieces {
    }  // namespace game_pieces

  }    // namespace blue_alliance

  namespace red_alliance {
    namespace game_pieces {
    }  // namespace game_pieces

  }  // namespace red_alliance
}  // namespace field_points
