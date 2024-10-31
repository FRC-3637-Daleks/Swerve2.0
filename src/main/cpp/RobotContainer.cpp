// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <units/math.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc/filter/SlewRateLimiter.h>

#include <choreo/lib/Choreo.h>

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

constexpr frc::Pose2d desiredPose{0_m, 0_m, 0_deg};

// Trapezoidal motion profile for the robot heading.
const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};
} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kDeadband = 0.08;
constexpr double kClimbDeadband = 0.08;

constexpr int kStrafeAxis = frc::Joystick::kDefaultXChannel;
constexpr int kForwardAxis = frc::Joystick::kDefaultYChannel;
constexpr int kRotationAxis = frc::Joystick::kDefaultTwistChannel;
constexpr int kThrottleAxis = frc::Joystick::kDefaultThrottleChannel;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr auto kMaxTeleopSpeed = 15.7_fps;
constexpr auto kMaxTeleopTurnSpeed = 2.5 * std::numbers::pi * 1_rad_per_s;

} // namespace OperatorConstants

namespace FieldConstants {

constexpr auto field_length = 54_ft + 3.25_in;
constexpr auto field_width = 26_ft + 11.75_in;
constexpr auto mid_line = field_length / 2;

} // namespace FieldConstants


RobotContainer::RobotContainer()
  : m_swerveController(OperatorConstants::kSwerveControllerPort)
  {

  fmt::println("made it to robot container");
  // Initialize all of your commands and subsystems here
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  // Log Match Info
  std::string matchType =
      frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone
          ? ""
          : (frc::DriverStation::GetMatchType() ==
                     frc::DriverStation::MatchType::kElimination
                 ? "Elimination"
                 : (frc::DriverStation::GetMatchType() ==
                            frc::DriverStation::MatchType::kQualification
                        ? "Qualification"
                        : "Practice"));

  std::string alliance =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed
           ? "Red"
           : "Blue");

  frc::DataLogManager::Log(
      fmt::format("Playing {} Match {} at {} as {} alliance\n", matchType,
                  frc::DriverStation::GetMatchNumber(),
                  frc::DriverStation::GetEventName(), alliance));

  // Configure the button bindings
  ConfigureBindings();

  // Configure Dashboard
  ConfigureDashboard();

  // Configure Auton.
  ConfigureAuto();

  frc::DataLogManager::Log(fmt::format("Finished initializing robot."));
}

void RobotContainer::ConfigureBindings() {
    auto throttle = [this]() -> double { 
    double input = m_swerveController.GetRawAxis(OperatorConstants::kThrottleAxis);
    double ret = ((-input +1))/2;
    return ret;
  };
  // Configure Swerve Bindings.
  auto fwd = [this, throttle]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(
        -m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis),
        OperatorConstants::kDeadband);
    auto squaredInput =
        input * std::abs(input); // square the input while preserving the sign
    auto alliance_flip = IsRed()? -1:1;
    return OperatorConstants::kMaxTeleopSpeed 
      * squaredInput
      * alliance_flip
      * throttle();
  };

  auto strafe = [this, throttle]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(
        -m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis),
        OperatorConstants::kDeadband);
    auto squaredInput = input * std::abs(input);
    auto alliance_flip = IsRed()? -1:1;
    return OperatorConstants::kMaxTeleopSpeed
      * squaredInput
      * alliance_flip
      * throttle();
  };

  auto rot = [this, throttle]() -> units::revolutions_per_minute_t {
    auto input = frc::ApplyDeadband(
        -m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis),
        OperatorConstants::kDeadband);
    auto squaredInput = input * std::abs(input);
    return OperatorConstants::kMaxTeleopTurnSpeed
      * squaredInput
      * throttle();
  };

  m_swerve.SetDefaultCommand(
      m_swerve.CustomSwerveCommand(fwd, strafe, rot));
  
  m_swerveController.Button(12).OnTrue(m_swerve.ZeroHeadingCommand());

  DriveToPoseTrigger.WhileTrue(
    m_swerve.DriveToPoseIndefinitelyCommand(AutoConstants::desiredPose));
  auto traj = choreolib::Choreo::GetTrajectory("Gerome");

  FollowPathTrigger.OnTrue(m_swerve.FollowPathCommand(traj));

}

void RobotContainer::ConfigureDashboard() {
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);
}

void RobotContainer::ConfigureAuto() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand() {
  return frc2::cmd::None();
}

bool RobotContainer::IsRed()
{
  m_isRed = (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed);

  return m_isRed;
}