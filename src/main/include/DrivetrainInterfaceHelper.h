#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <concepts>

template<typename T>
concept Distance = units::traits::is_length_unit_v<T>;

template<typename T>
concept LinearPositionSupplier = requires(T a)
{
  { a() } -> Distance;
};

template<typename T>
concept Rotation = units::traits::is_angle_unit_v<T>;

template<typename T>
concept RotationSupplier = requires(T a)
{
  { a() } -> Rotation;
};

template<typename T>
concept LinearVelocity = units::traits::is_velocity_unit_v<T>;

template<typename T>
concept LinearVelocitySupplier = requires(T a)
{
  { a() } -> LinearVelocity;
};

template<typename T>
concept AngularVelocity = units::traits::is_angular_velocity_unit_v<T>;

template<typename T>
concept AngularVelocitySupplier = requires(T a)
{
  { a() } -> AngularVelocity;
};

template<typename T>
concept LinearCmd = 
     Distance<T>
  || LinearVelocity<T>
  || LinearPositionSupplier<T>
  || LinearVelocitySupplier<T>;

template<typename T>
concept RotationCmd = 
     Rotation<T>
  || AngularVelocity<T>
  || RotationSupplier<T>
  || AngularVelocitySupplier<T>;
