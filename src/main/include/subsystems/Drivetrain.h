#pragma once

#include <AHRS.h>
#include <frc/PowerDistribution.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/HolonomicDriveController.h>

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <memory>
#include <numbers>
#include <utility>

#include "SwerveModule.h"

// Forward Declaration
class DrivetrainSimulation;

/**
 * The Drivetrain subsystem contains four swerve modules and a gyroscope. The
 * Drivetrain can be driven at a certain speed represented as a 2-dimensional
 * translation on the field. This translation can be commanded either relative
 * to the front of the robot, or relative to a certain absolute position. These
 * two modes are called "robot relative" and "field relative" respectively. The
 * drivetrain can also be commanded to rotate at a given speed independent of
 * its translation.
 */
class Drivetrain : public frc2::SubsystemBase {
public:
  // The ctor of the Drivetrain subsystem.
  Drivetrain();

  // Need to define destructor to make simulation code compile
  ~Drivetrain();

  // Updates the odometer and SmartDashboard.
  void Periodic() override;

  // Executes the simulation
  void SimulationPeriodic() override;

  // Drives the robot at the given translational and rotational speed. Forward
  // is front-back movement, strafe is left-right movement. If fieldRelative is
  // set, the translation is relative to the field instead of the robot heading.
  void Drive(units::meters_per_second_t forwardSpeed,
             units::meters_per_second_t strafeSpeed,
             units::radians_per_second_t angularSpeed, bool fieldRelative,
             bool isRed);

  // Sets the state of each swerve module.
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  // Returns the heading of the robot.
  frc::Rotation2d GetHeading();

  frc::Rotation2d GetGyroHeading();

  // Zeroes the robot heading.
  void ZeroHeading();

  void ZeroAbsEncoders();

  void SetAbsEncoderOffset();

  void SyncEncoders();

  void CoastMode(bool coast);

  // Returns the rotational velocity of the robot in degrees per second.
  units::degrees_per_second_t GetTurnRate();

  // Returns the robot heading and translation as a Pose2d.
  frc::Pose2d GetPose();

  frc::Pose2d GetSimulatedGroundTruth();

  bool AtPose(frc::Pose2d desiredPose, frc::Pose2d tolerance);

  // Returns Current Chassis Speed
  frc::ChassisSpeeds GetChassisSpeed();

  units::meters_per_second_t GetSpeed();
  // Resets the odometry using the given a field-relative pose using current
  // gyro angle.
  void ResetOdometry(const frc::Pose2d &pose);

  frc::SwerveDriveKinematics<4> kDriveKinematics;

  // Display useful information on Shuffleboard.
  void UpdateDashboard();
  frc::Field2d &GetField() { return m_field; }

  // Drive the robot with swerve controls.
  frc2::CommandPtr
  SwerveCommand(std::function<units::meters_per_second_t()> forward,
                std::function<units::meters_per_second_t()> strafe,
                std::function<units::revolutions_per_minute_t()> rot);

  // Drive the robot with field-relative swerve controls.
  frc2::CommandPtr SwerveCommandFieldRelative(
      std::function<units::meters_per_second_t()> forward,
      std::function<units::meters_per_second_t()> strafe,
      std::function<units::revolutions_per_minute_t()> rot,
      std::function<bool()> isRed);

  frc2::CommandPtr
  SwerveSlowCommand(std::function<units::meters_per_second_t()> forward,
                    std::function<units::meters_per_second_t()> strafe,
                    std::function<units::revolutions_per_minute_t()> rot,
                    std::function<bool()> isRed);

  // frc2::CommandPtr FollowPathCommand(frc::Pose2d desiredPose,
  //                                     std::vector<frc::Translation2d> waypoints,
  //                                     units::meters_per_second_t maxSpeed,
  //                                     units::meters_per_second_squared_t maxAccel,
  //                                     units::radians_per_second_t maxAngularSpeed,
  //                                     units::radians_per_second_squared_t maxAngularAccel,
  //                                     bool isRed);

frc2::CommandPtr DriveToPoseCommand(
    frc::Pose2d desiredPose,
    bool isRed,
    units::meters_per_second_t endVelo = units::meters_per_second_t(0.0),
    frc::Pose2d tolerance = frc::Pose2d(0.06_m, 0.06_m, 3_deg));

  // Returns a command that zeroes the robot heading.
  frc2::CommandPtr ZeroHeadingCommand();

  frc2::CommandPtr ZeroAbsEncodersCommand();

  frc2::CommandPtr SetAbsEncoderOffsetCommand();

  frc2::CommandPtr CoastModeCommand(bool coast);

  frc2::CommandPtr ConfigAbsEncoderCommand();

  // Returns a command that stops the robot.
  frc2::CommandPtr BrakeCommand();

  frc2::CommandPtr TurnToAngleCommand(units::degree_t angle);

private:
  // magic to make doing stuff for every module easier
  auto each_module(auto&& fn)
  {
    return std::apply([&fn](auto&&... ms)
    {
      return wpi::array{
        std::forward<decltype(fn)>(fn)(
          std::forward<decltype(ms)>(ms))...};
    }, m_modules);
  }

  auto each_position() {
    return each_module([](SwerveModule &m) {return m.GetPosition();});
  }

  auto each_state() {
    return each_module([](SwerveModule &m) {return m.GetState();});
  }

private:
  enum module_id {
    kFrontLeft=0, kFrontRight, kRearLeft, kRearRight, kNumModules};
  std::array<SwerveModule, kNumModules> m_modules;

  AHRS m_gyro;

  frc::PowerDistribution m_pdh;

  // Pose Estimator for estimating the robot's position on the field.
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Field widget for Shuffleboard.
  frc::Field2d m_field;

  frc::ProfiledPIDController<units::degrees> m_turnPID;
  frc::ProfiledPIDController<units::radians> m_thetaPID;
  frc::PIDController m_XYController;
  frc::HolonomicDriveController m_holonomicController;
  frc2::CommandPtr zeroEncodersCommand{ZeroAbsEncodersCommand()};

private:
  friend class DrivetrainSimulation;
  std::unique_ptr<DrivetrainSimulation> m_sim_state;
};