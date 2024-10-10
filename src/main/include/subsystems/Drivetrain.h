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
#include "DrivetrainInterfaceHelper.h"

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
private:
  enum module_id {
    kFrontLeft=0, kFrontRight, kRearLeft, kRearRight, kNumModules
  };

public:
  using module_states_t = 
    wpi::array<frc::SwerveModuleState, kNumModules>;
  
  using linear_cmd_supplier_t =
    std::function<units::meters_per_second_t()>;
  
  using rotation_cmd_supplier_t =
    std::function<units::revolutions_per_minute_t()>;
  
  using pose_supplier_t =
    std::function<frc::Pose2d()>;

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
  // is front-back movement, strafe is left-right movement.
  // Motion is relative to the robot's frame
  void Drive(const frc::ChassisSpeeds &cmd_vel);
  void DriveFieldRelative(const frc::ChassisSpeeds &cmd_vel);

  // Sets the state of each swerve module.
  void SetModuleStates(const module_states_t &desiredStates);

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

  bool AtPose(
    const frc::Pose2d &desiredPose,
    const frc::Pose2d &tolerance={0.06_m, 0.06_m, 2_deg});

  // Returns Current Chassis Speed
  frc::ChassisSpeeds GetChassisSpeed();

  units::meters_per_second_t GetSpeed();
  // Resets the odometry using the given a field-relative pose using current
  // gyro angle.
  void ResetOdometry(const frc::Pose2d &pose);

  // Display useful information on Shuffleboard.
  void InitializeDashboard();
  void UpdateDashboard();
  frc::Field2d &GetField() { return m_field; }

  // Drive the robot with swerve controls.
  frc2::CommandPtr SwerveCommand(
    linear_cmd_supplier_t forward,
    linear_cmd_supplier_t strafe,
    rotation_cmd_supplier_t rot);

  // Drive the robot with field-relative swerve controls.
  frc2::CommandPtr SwerveCommandFieldRelative(
    linear_cmd_supplier_t forward,
    linear_cmd_supplier_t strafe,
    rotation_cmd_supplier_t rot);

  // Drives the robot to 'desiredPose()' with feedforward 'endVelo'
  // until its within 'tolerance' of 'desiredPose'
  frc2::CommandPtr DriveToPoseCommand(
    pose_supplier_t desiredPoseSupplier,
    units::meters_per_second_t endVelo = 0.0_mps,
    const frc::Pose2d &tolerance = {0.06_m, 0.06_m, 3_deg});
  
  frc2::CommandPtr DriveToPoseCommand(
    const frc::Pose2d &desiredPose,
    units::meters_per_second_t endVelo = 0.0_mps,
    const frc::Pose2d &tolerance = {0.06_m, 0.06_m, 3_deg}) {
    return DriveToPoseCommand(
      [desiredPose] {return desiredPose;}, endVelo, tolerance);
  }
  
  // Drives the robot toward 'desiredPose()'
  // Warning: This command will not terminate unless interrupted,
  // or until the specified timeout.
  // Should be bound to triggers with WhileTrue
  frc2::CommandPtr DriveToPoseIndefinitelyCommand(
    pose_supplier_t desiredPoseSupplier,
    units::second_t timeout = 3.0_s) {
    return DriveToPoseCommand(std::move(desiredPoseSupplier), 0.0_mps, {}).WithTimeout(timeout);
  }

  frc2::CommandPtr DriveToPoseIndefinitelyCommand(
    const frc::Pose2d &desiredPose,
    units::second_t timeout = 3.0_s) {
    return DriveToPoseIndefinitelyCommand(
      [desiredPose] {return desiredPose;}, timeout);
  }
  
  // Cmd here refers to the commanded motion, not wpilib commands 
  frc2::CommandPtr AssistedDriveCommand(
    LinearCmd auto&& x_cmd,
    LinearCmd auto&& y_cmd,
    RotationCmd auto&& theta_cmd) {
    return SwerveCommandFieldRelative(
      x_speed(std::forward<decltype(x_cmd)>(x_cmd)),
      y_speed(std::forward<decltype(y_cmd)>(y_cmd)),
      theta_speed(std::forward<decltype(theta_cmd)>(theta_cmd))
    );
  }

  frc2::CommandPtr ZTargetCommand(
    linear_cmd_supplier_t fwd,
    linear_cmd_supplier_t strafe,
    pose_supplier_t target) {
    return AssistedDriveCommand(
      std::move(fwd),
      std::move(strafe),
      [this, target] {
        return (target().Translation() - GetPose().Translation()).Angle().Radians();
      }
    );
  }

  // Returns a command that zeroes the robot heading.
  frc2::CommandPtr ZeroHeadingCommand();

  frc2::CommandPtr ZeroAbsEncodersCommand();

  frc2::CommandPtr SetAbsEncoderOffsetCommand();

  frc2::CommandPtr CoastModeCommand(bool coast);

  frc2::CommandPtr ConfigAbsEncoderCommand();

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
  // By defining these for several input types, AssistedDriveCommand
  // can take in any permutation of static setpoints, dynamic setpoints,
  // or dynamic control inputs and generate a command that does it

  // Runs the PID controller for a dynamic position
  linear_cmd_supplier_t x_speed(LinearPositionSupplier auto &&position) {
    return [this, position = std::forward<decltype(position)>(position)] {
      return units::meters_per_second_t{
        m_holonomicController.getXController().Calculate(
          units::meter_t{GetPose().X()}.value(),
          std::forward<decltype(position)>(position)().value())
      };
    };
  }
  linear_cmd_supplier_t y_speed(LinearPositionSupplier auto &&position) {
    return [this, position = std::forward<decltype(position)>(position)] {
      return units::meters_per_second_t{
        m_holonomicController.getYController().Calculate(
          units::meter_t{GetPose().Y()}.value(),
          std::forward<decltype(position)>(position)().value())
      };
    };
  }
  rotation_cmd_supplier_t theta_speed(RotationSupplier auto &&heading) {
    return [this, heading = std::forward<decltype(heading)>(heading)] {
      return units::radians_per_second_t{
        m_holonomicController.getThetaController().Calculate(
          GetPose().Rotation().Radians(),
          std::forward<decltype(heading)>(heading)())
      };
    };
  }

  // Runs the PID controller for a static position
  linear_cmd_supplier_t x_speed(Distance auto position)
  {return x_speed([position] {return position;});}
  linear_cmd_supplier_t y_speed(Distance auto position)
  {return y_speed([position] {return position;});}
  rotation_cmd_supplier_t theta_speed(Rotation auto heading)
  {return theta_speed([heading] {return heading;});}

  // Passes through the speed
  linear_cmd_supplier_t x_speed(LinearVelocitySupplier auto &&velocity)
  {return std::forward<decltype(velocity)>(velocity);}
  linear_cmd_supplier_t y_speed(LinearVelocitySupplier auto &&velocity)
  {return std::forward<decltype(velocity)>(velocity);}
  rotation_cmd_supplier_t theta_speed(AngularVelocitySupplier auto &&velocity)
  {return std::forward<decltype(velocity)>(velocity);}

  linear_cmd_supplier_t x_speed(LinearVelocity auto velocity)
  {return [velocity] {return velocity;};}
  linear_cmd_supplier_t y_speed(LinearVelocity auto velocity)
  {return [velocity] {return velocity;};}
  linear_cmd_supplier_t theta_speed(AngularVelocity auto velocity)
  {return [velocity] {return velocity;};}

private:
  frc::SwerveDriveKinematics<4> kDriveKinematics;
  std::array<SwerveModule, kNumModules> m_modules;

  AHRS m_gyro;

  frc::PowerDistribution m_pdh;

  // Pose Estimator for estimating the robot's position on the field.
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Field widget for Shuffleboard.
  frc::Field2d m_field;

  // Stores controllers for each motion axis
  frc::HolonomicDriveController m_holonomicController;
  
  // For placement on Dashboard
  frc2::CommandPtr zeroEncodersCommand{ZeroAbsEncodersCommand()};

private:
  friend class DrivetrainSimulation;
  std::unique_ptr<DrivetrainSimulation> m_sim_state;
};
