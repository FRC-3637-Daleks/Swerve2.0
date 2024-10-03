// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/JoystickButton.h>

#include "frc/apriltag/AprilTagFields.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "subsystems/Drivetrain.h"

namespace AutoConstants {

constexpr auto kMaxSpeed = 4.5_mps;
constexpr auto kMaxAcceleration = 6_mps_sq;
constexpr auto kPathMaxAcceleration = 4_mps_sq;
// Swerve Constants (NEED TO BE INTEGRATED)
// constexpr auto kMaxSpeed = ModuleConstants::kPhysicalMaxSpeed / 3; // left
// out as these are repeat values constexpr auto kMaxAcceleration = 10_fps_sq;
constexpr auto kMaxAngularSpeed = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s_sq;

// XXX Very untrustworthy placeholder values.
constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

constexpr frc::Pose2d desiredPose{8.3_m, .77_m, 0_deg};
constexpr frc::Translation2d waypoint1{8.3_m, 4.1_m};
constexpr frc::Translation2d waypoint2{8.3_m, 5.78_m};

const std::vector<frc::Translation2d> waypointzVector{waypoint1, waypoint2};


// Trapezoidal motion profile for the robot heading.
const frc::TrapezoidProfile<units::radians>::Constraints
  kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};
} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kDeadband = 0.12;
constexpr double kClimbDeadband = 0.08;

constexpr int kStrafeAxis = frc::Joystick::kXAxis;
constexpr int kForwardAxis = frc::Joystick::kYAxis;
constexpr int kRotationAxis = frc::Joystick::kZAxis;
constexpr int kThrottleAxis = frc::Joystick::kThrottleAxis;
} // namespace OperatorConstants

namespace FieldConstants {

constexpr auto field_length = 54_ft + 3.25_in;
constexpr auto field_width = 26_ft + 11.75_in;
constexpr auto mid_line = field_length / 2;

} // namespace FieldConstants

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetDisabledCommand();
  frc2::CommandPtr GetAutonomousCommand();



public:
  // Replace with CommandPS4Controller or CommandJoystick if needed

  frc2::CommandJoystick m_swerveController{
      OperatorConstants::kSwerveControllerPort};

  frc2::Trigger DriveToPoseTrigger{[this]() -> bool {
    return m_swerveController.GetPOV() == 90; //right on DPad
  }};
  // The robot's subsystems are defined here...

  Drivetrain m_swerve;

  bool m_isRed;

public:
  void ConfigureBindings();
  void ConfigureDashboard();
  void ConfigureAuto();
};