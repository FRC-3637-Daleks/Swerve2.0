#include "subsystems/Drivetrain.h"

#include <frc/DataLogManager.h>
#include <frc/I2C.h>
#include <frc/SPI.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>
#include <wpi/array.h>

#include <units/time.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/force.h>
#include <units/math.h>

#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

#include <numeric>
#include <iostream>

namespace DriveConstants {
constexpr auto kMaxSpeed = 15.7_fps;
constexpr auto kMaxAccel = 6_mps_sq;
constexpr auto kWeight = 123_lb;
constexpr auto kMaxTurnRate = 2.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 6 * std::numbers::pi * 1_rad_per_s_sq;

constexpr auto kPeriod = 20_ms;

constexpr double kPTheta = 3.62;
constexpr double kITheta = 0.00;
constexpr double kDTheta = 0.00;
inline const frc::ProfiledPIDController<units::radians> kThetaPID{
  kPTheta, kITheta, kDTheta,
  {kMaxTurnRate, kMaxTurnAcceleration}
};

constexpr double kPXY = 5.2;
constexpr double kIXY = 0.0;
constexpr double kDXY= 0.0;
inline const frc::PIDController kTranslatePID{
  kPXY, kIXY, kDXY
};

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

class DrivetrainSimulation {
public:
  DrivetrainSimulation(Drivetrain &drivetrain)
      : m_gyroYaw(HALSIM_GetSimValueHandle(
            HALSIM_GetSimDeviceHandle("navX-Sensor[4]"), "Yaw")),
        m_poseSim(drivetrain.kDriveKinematics, drivetrain.GetHeading(),
                  drivetrain.each_position()) {}

public:
  hal::SimDouble m_gyroYaw;
  frc::SwerveDriveOdometry<4> m_poseSim;
};

Drivetrain::Drivetrain()
    : kDriveKinematics{
        frc::Translation2d{ kWheelBase/2,  kTrackWidth/2},
        frc::Translation2d{ -kWheelBase/2, kTrackWidth/2},
        frc::Translation2d{ kWheelBase/2,  -kTrackWidth/2},
        frc::Translation2d{-kWheelBase/2, -kTrackWidth/2}},
      m_modules{{
        {"FL",
          kFrontLeftDriveMotorId,
          kFrontLeftSteerMotorId,
          kFrontLeftAbsoluteEncoderChannel},
        {"RL",
          kRearLeftDriveMotorId,
          kRearLeftSteerMotorId,
          kRearLeftAbsoluteEncoderChannel},
        {"FR",
          kFrontRightDriveMotorId,
          kFrontRightSteerMotorId,
          kFrontRightAbsoluteEncoderChannel},
        {"RR",
          kRearRightDriveMotorId,
          kRearRightSteerMotorId,
          kRearRightAbsoluteEncoderChannel}
      }},
      m_gyro{frc::SPI::Port::kMXP},
      m_pdh{kPDH, frc::PowerDistribution::ModuleType::kRev},
      m_poseEstimator{
        kDriveKinematics, GetGyroHeading(), each_position(), frc::Pose2d()},
      m_holonomicController(kTranslatePID, kTranslatePID, kThetaPID),
      m_trajConfig(kMaxSpeed, kMaxAccel),
      m_sim_state(new DrivetrainSimulation(*this))  {
  
  InitializeDashboard();

  frc::DataLogManager::Log(
      fmt::format("Finished initializing drivetrain subsystem."));
}

frc::Pose2d Drivetrain::GetSimulatedGroundTruth() {
  return m_sim_state->m_poseSim.GetPose();
}

void Drivetrain::Periodic() {

  // Do this once per loop
  SwerveModule::RefreshAllSignals(m_modules);

  // Update the odometry with the current gyro angle and module states.
  m_poseEstimator.Update(GetGyroHeading(), each_position());

  this->UpdateDashboard();
}

Drivetrain::~Drivetrain() {}

void Drivetrain::RobotRelativeDrive(const frc::ChassisSpeeds &cmd_vel) {
  auto states = kDriveKinematics.ToSwerveModuleStates(cmd_vel);

  // Occasionally a drive motor is commanded to go faster than its maximum
  // output can sustain. Desaturation lowers the module speeds so that no motor
  // is driven above its maximum speed, while preserving the intended motion.
  kDriveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kPhysicalMaxSpeed);

  // Finally each of the desired states can be sent as commands to the modules.
  SetModuleStates(states);
}

void Drivetrain::Drive(const frc::ChassisSpeeds &cmd_vel) {
  RobotRelativeDrive(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      cmd_vel, GetHeading()));
}

void Drivetrain::SetModuleStates(
    const wpi::array<frc::SwerveModuleState, kNumModules> &desiredStates) {
  for (int i = 0; i < kNumModules; i++)
    m_modules[i].SetDesiredState(desiredStates[i]);
}

frc::Rotation2d Drivetrain::GetHeading() { return GetPose().Rotation(); }

frc::Rotation2d Drivetrain::GetGyroHeading() {
  return units::degree_t(-m_gyro.GetYaw());
}

void Drivetrain::ZeroHeading() {
  auto pose = GetPose();
  ResetOdometry(frc::Pose2d{pose.X(), pose.Y(), 0_deg});
}

void Drivetrain::ZeroAbsEncoders() {
  for (auto &m : m_modules) m.ZeroAbsEncoders();
}

void Drivetrain::SetAbsEncoderOffset() {
  for (auto &m : m_modules) m.SetEncoderOffset();
}

void Drivetrain::SyncEncoders() {
  for (auto &m : m_modules) m.SyncEncoders();
}

void Drivetrain::CoastMode(bool coast) {
  for (auto &m : m_modules) m.CoastMode(coast);
}

void Drivetrain::DriveToPose(
    const frc::Trajectory::State &state,
    const frc::Pose2d &tolerance)   {
      auto currentPose = GetPose();
      m_holonomicController.SetTolerance(tolerance);
      const auto speeds = m_holonomicController.Calculate(
          currentPose, state, state.pose.Rotation());
    RobotRelativeDrive(speeds);
};

void Drivetrain::DriveToPose(
    pose_supplier_t desiredPoseSupplier,
    units::meters_per_second_t endVelo,
    const frc::Pose2d &tolerance) {
      auto desiredPose = desiredPoseSupplier();
      auto currentPose = GetPose();
      auto desiredRot = desiredPose.Rotation();
      m_holonomicController.SetTolerance(tolerance);
      m_field.GetObject("Desired Pose")->SetPose(desiredPose);
      const auto speeds = m_holonomicController.Calculate(
          currentPose, desiredPose, endVelo, desiredRot);
      RobotRelativeDrive(speeds);
    }
    
units::degrees_per_second_t Drivetrain::GetTurnRate() {
  return -m_gyro.GetRate() * 1_deg_per_s;
}

frc::Pose2d Drivetrain::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

frc::ChassisSpeeds Drivetrain::GetChassisSpeed() {
  return kDriveKinematics.ToChassisSpeeds(each_state());
}

units::meters_per_second_t Drivetrain::GetSpeed(){
  const auto speeds = GetChassisSpeed();
  return units::math::sqrt(speeds.vx*speeds.vx + speeds.vy*speeds.vy);
}

bool Drivetrain::AtPose(const frc::Pose2d &desiredPose, const frc::Pose2d &tolerance,
                        const units::meters_per_second_t endVelo) {
  auto currentPose = GetPose();
  auto poseError = currentPose.RelativeTo(desiredPose);
  return (units::math::abs(poseError.X()) < tolerance.X()) &&
         (units::math::abs(poseError.Y()) < tolerance.Y()) &&
         (units::math::abs(poseError.Rotation().Degrees()) < tolerance.Rotation().Degrees()) &&
         (GetSpeed() < endVelo);
} 

void Drivetrain::ResetOdometry(const frc::Pose2d &pose) {
  m_poseEstimator.ResetPosition(
      GetGyroHeading(), each_position(), pose);
}

void Drivetrain::InitializeDashboard() {
  // PutData is persistent, these objects only need to be passed in once
  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("zeroEncodersCommand",
                               zeroEncodersCommand.get());
  frc::SmartDashboard::PutData("PDH", &m_pdh);

  frc::SmartDashboard::PutData("Swerve/ThetaPIDController",
                               &m_holonomicController.getThetaController());
  frc::SmartDashboard::PutData("Swerve/XPIDController",
                               &m_holonomicController.getXController());
  frc::SmartDashboard::PutData("Swerve/YPIDController",
                               &m_holonomicController.getYController());
}

void Drivetrain::UpdateDashboard() {
  const auto robot_center = this->GetPose();
  
  m_field.SetRobotPose(this->GetPose());

  constexpr int xs[] = {1, 1, -1, -1};
  constexpr int ys[] = {1, -1, 1, -1};
  for (int i = 0; i < kNumModules; i++) {
    const auto module_pose = robot_center.TransformBy(
      {xs[i]*kWheelBase/2, ys[i]*kTrackWidth/2, m_modules[i].GetState().angle});
    m_field.GetObject(m_modules[i].GetName())->SetPose(module_pose);
  }

  frc::SmartDashboard::PutBoolean("Swerve/Gyro calibrating?",
                                  m_gyro.IsCalibrating());
  frc::SmartDashboard::PutNumber("Swerve/Robot heading",
                                 GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("Robot Speed (mps)",
                                 units::meters_per_second_t{GetSpeed()}.value());

  for (auto &m : m_modules) m.UpdateDashboard();

  frc::SmartDashboard::PutNumber("Swerve/Gyro", m_gyro.GetAngle());
  
  double error[] = {
    m_holonomicController.getXController().GetPositionError(),
    m_holonomicController.getYController().GetPositionError(),
    m_holonomicController.getThetaController().GetPositionError().value()};
  frc::SmartDashboard::PutNumberArray("Error", error);
}

frc2::CommandPtr Drivetrain::RobotRelativeSwerveCommand(
    chassis_speed_supplier_t cmd_vel) {
  return this->Run([=, this] {
    RobotRelativeDrive(cmd_vel());
  });
}

frc2::CommandPtr Drivetrain::BasicSwerveCommand(
    chassis_speed_supplier_t cmd_vel) {
  return this->Run([=, this] {
    Drive(cmd_vel());
  });
}
frc::Trajectory Drivetrain::trajGenerator(
    pose_supplier_t desiredPoseSupplier,
    const std::vector<frc::Translation2d> &waypoints)
    {
      return
        frc::TrajectoryGenerator::GenerateTrajectory({0.0_m, 0.0_m, 0_deg},
        waypoints, desiredPoseSupplier(), m_trajConfig);
    }

// frc2::CommandPtr Drivetrain::FollowPathCommand(
//     pose_supplier_t desiredPoseSupplier,
//     const std::vector<frc::Translation2d> &waypoints,
//     units::meters_per_second_t endVelo,
//     const frc::Pose2d &tolerance)   {
//     auto trajectory = std::make_shared<frc::Trajectory>();
//     return this->RunOnce([=, this] {
//       auto currentPose = GetPose();
//       auto desiredPose = desiredPoseSupplier();
//       auto desiredRot = desiredPose.Rotation();
//       m_trajConfig.SetKinematics(kDriveKinematics);
//       m_trajConfig.SetStartVelocity(GetSpeed());
//       m_trajConfig.SetEndVelocity(endVelo);
//       *trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//         currentPose, waypoints, desiredPose, m_trajConfig);
//     }).AndThen(Run(
//     [=, this] {
//       m_field.GetObject("Robot Traj")->SetTrajectory(*trajectory);
//       m_field.GetObject("Traj Pose")->SetPose(trajectory->States()[100].pose);
//       auto states = trajectory->States();
//       for(auto state : states)
//       DriveToPose(state, tolerance);
//     }));
// }

frc2::CommandPtr Drivetrain::ZeroHeadingCommand() {
  return this->RunOnce([&] { ZeroHeading(); });
}

frc2::CommandPtr Drivetrain::ZeroAbsEncodersCommand() {
  return this
      ->RunOnce([&] {
        ZeroAbsEncoders();
      })
      .IgnoringDisable(true);
};

frc2::CommandPtr Drivetrain::SetAbsEncoderOffsetCommand() {
  return this->RunOnce([this] { SetAbsEncoderOffset(); }).IgnoringDisable(true);
}

frc2::CommandPtr Drivetrain::CoastModeCommand(bool coast) {
  return this->StartEnd([this] { this->CoastMode(true); },
                        [this] { this->CoastMode(false); });
}

frc2::CommandPtr Drivetrain::ConfigAbsEncoderCommand() {
  return this
      ->StartEnd(
          [this] {
            CoastMode(true);
            ZeroAbsEncoders();
          },
          [this] {
            CoastMode(false);
            SetAbsEncoderOffset();
          })
      .AndThen(frc2::WaitCommand(0.5_s).ToPtr())
      .AndThen(this->RunOnce([this] { SyncEncoders(); }))
      .IgnoringDisable(true);
}

void Drivetrain::SimulationPeriodic() {
  if (!m_sim_state)
    return;

  for (auto &m : m_modules) m.SimulationPeriodic();

  // Assume perfect kinematics and get the new gyro angle
  const auto chassis_speed = kDriveKinematics.ToChassisSpeeds(each_state());
  const auto theta = m_sim_state->m_poseSim.GetPose().Rotation();
  const auto new_theta =
      theta.RotateBy(units::radian_t{chassis_speed.omega * 20_ms});
  // robot nav x defines clockwise as positive instead of counterclockwise
  m_sim_state->m_gyroYaw.Set(-new_theta.Degrees().value());

  // Feed this simulated gyro angle into the odometry to get simulated position
  m_sim_state->m_poseSim.Update(new_theta, each_position());

  m_field.GetObject("simulation")->SetPose(m_sim_state->m_poseSim.GetPose());
}