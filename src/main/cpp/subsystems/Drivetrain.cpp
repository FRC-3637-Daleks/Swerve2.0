#include "subsystems/Drivetrain.h"

#include <frc/DataLogManager.h>
#include <frc/I2C.h>
#include <frc/SPI.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <wpi/array.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/force.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

#include <iostream>
#include <numeric>

#include <swerve/PathFollower.h>

namespace DriveConstants {
constexpr auto kMaxSpeed = 15.7_fps;
constexpr auto kMaxAccel = 6_mps_sq;
constexpr auto kWeight = 70_lb;
constexpr auto kBatteryWeight = 12.9_lb;
constexpr auto kMaxTurnRate = 2.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 6 * std::numbers::pi * 1_rad_per_s_sq;

constexpr auto kPeriod = 20_ms;
constexpr auto kOdomPeriod = 5_ms;
constexpr int kOdomHertz = 200;

constexpr double kPTheta = 3.62;
constexpr double kITheta = 0.00;
constexpr double kDTheta = 0.00;
inline const frc::ProfiledPIDController<units::radians> kThetaPID{
    kPTheta, kITheta, kDTheta, {kMaxTurnRate, kMaxTurnAcceleration}};

constexpr double kPXY = 5.2;
constexpr double kIXY = 0.0;
constexpr double kDXY = 0.0;
inline const frc::PIDController kTranslatePID{kPXY, kIXY, kDXY};

// Swerve Constants
constexpr auto kTrackWidth =
25_in; // Distance between centers of right and left wheels.
constexpr auto kWheelBase =
25_in; // Distance between centers of front and back wheels.
const auto kRadius = units::meter_t(std::sqrt(.91));

constexpr int kFrontLeftDriveMotorId = 1;
constexpr int kRearLeftDriveMotorId = 3;
constexpr int kFrontRightDriveMotorId = 5;
constexpr int kRearRightDriveMotorId = 7;

constexpr int kFrontLeftSteerMotorId = 2;
constexpr int kRearLeftSteerMotorId = 4;
constexpr int kFrontRightSteerMotorId = 6;
constexpr int kRearRightSteerMotorId = 8;

constexpr int kFrontLeftAbsoluteEncoderChannel = 9;
constexpr int kRearLeftAbsoluteEncoderChannel = 10;
constexpr int kFrontRightAbsoluteEncoderChannel = 11;
constexpr int kRearRightAbsoluteEncoderChannel = 12;

constexpr int kPDH = 25;

} // namespace DriveConstants

using namespace DriveConstants;

Drivetrain::Drivetrain() {
  frc::SmartDashboard::PutData(resetOdomCommand.get());
  frc::DataLogManager::Log(
    fmt::format("Finished initializing drivetrain subsystem."));
}

Drivetrain::~Drivetrain() {}

frc2::CommandPtr Drivetrain::DynamicOdomReset() {
  return this->RunOnce([=, this] {
    auto reset_point = GetField().GetObject("reset_point")->GetPose();
    fmt::println("Resetting Odom to: {}, {}, {}", reset_point.X(),
      reset_point.Y(), reset_point.Rotation().Radians());
    ResetOdometry(reset_point);
    })
    .IgnoringDisable(true);
}

frc2::CommandPtr
Drivetrain::RobotRelativeSwerveCommand(chassis_speed_supplier_t cmd_vel) {
  return this->Run([=, this] { RobotRelativeDrive(cmd_vel()); });
}

frc2::CommandPtr
Drivetrain::BasicSwerveCommand(chassis_speed_supplier_t cmd_vel) {
  return this->Run([=, this] { Drive(cmd_vel()); });
}

frc2::CommandPtr Drivetrain::ZeroHeadingCommand() {
  return this->RunOnce([&] { ZeroHeading(); });
}

frc2::CommandPtr Drivetrain::CoastModeCommand(bool coast) {
  return this->StartEnd([this] { this->CoastMode(true); },
    [this] { this->CoastMode(false); });
}

frc2::CommandPtr Drivetrain::FollowPathCommand(
  choreo::Trajectory<choreo::SwerveSample> trajectory) {
  return PathFollower{std::move(trajectory), *this}.ToPtr();
}