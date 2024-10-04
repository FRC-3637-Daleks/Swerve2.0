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
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
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

#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

namespace DriveConstants {
constexpr auto kMaxSpeed = 15.7_fps;
constexpr auto kWeight = 123_lb;
constexpr auto kMaxTurnRate = 2.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 6 * std::numbers::pi * 1_rad_per_s_sq;

constexpr auto kPeriod = 20_ms;

constexpr double kPTurn = 0.071;
constexpr double kITurn = 0.00;
constexpr double kDTurn = 0.00;

// Swerve Constants
constexpr auto kTrackWidth =
    25_in; // Distance between centers of right and left wheels.
constexpr auto kWheelBase =
    25_in; // Distance between centers of front and back wheels.
// const auto kRadius = 19.5_in; // 19.5 inches
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
        frc::Translation2d{ kWheelBase/2, -kTrackWidth/2},
        frc::Translation2d{-kWheelBase/2,  kTrackWidth/2},
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
        kDriveKinematics, GetGyroHeading(), each_position(),
        frc::Pose2d()},
      m_turnPID{kPTurn, kITurn, kDTurn,
        {kMaxTurnRate, kMaxTurnAcceleration}},
      m_sim_state(new DrivetrainSimulation(*this)) {

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

void Drivetrain::Drive(units::meters_per_second_t forwardSpeed,
                       units::meters_per_second_t strafeSpeed,
                       units::radians_per_second_t angularSpeed,
                       bool fieldRelative, bool isRed) {

    auto states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});

  if (fieldRelative) {
    if (isRed)  
      states = kDriveKinematics.ToSwerveModuleStates(
          frc::ChassisSpeeds::Discretize(
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              -1 * forwardSpeed, -1 * strafeSpeed, angularSpeed, GetHeading()), kPeriod));
    else
      states = kDriveKinematics.ToSwerveModuleStates(
        frc::ChassisSpeeds::Discretize(
          frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              forwardSpeed, strafeSpeed, angularSpeed, GetHeading()), kPeriod));
  } else {
    states = kDriveKinematics.ToSwerveModuleStates(
        frc::ChassisSpeeds{forwardSpeed, strafeSpeed, angularSpeed});
  }

  // Occasionally a drive motor is commanded to go faster than its maximum
  // output can sustain. Desaturation lowers the module speeds so that no motor
  // is driven above its maximum speed, while preserving the intended motion.
  kDriveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kPhysicalMaxSpeed);

  // Finally each of the desired states can be sent as commands to the modules.
  for (int i = 0; i < kNumModules; i++)
    m_modules[i].SetDesiredState(states[i]);
}

void Drivetrain::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
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

units::degrees_per_second_t Drivetrain::GetTurnRate() {
  return -m_gyro.GetRate() * 1_deg_per_s;
}

frc::Pose2d Drivetrain::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

frc::ChassisSpeeds Drivetrain::GetSpeed() {
  return kDriveKinematics.ToChassisSpeeds(each_state());
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose) {
  m_poseEstimator.ResetPosition(
      GetGyroHeading(), each_position(), pose);
}

void Drivetrain::UpdateDashboard() {
  const auto robot_center = this->GetPose();
  m_field.SetRobotPose(this->GetPose());

  int xs[] = {1, 1, -1, -1};
  int ys[] = {1, -1, 1, -1};
  for (int i = 0; i < kNumModules; i++) {
    const auto module_pose = robot_center.TransformBy(
      {xs[i]*kWheelBase/2, ys[i]*kTrackWidth/2, m_modules[i].GetState().angle});
    m_field.GetObject(m_modules[i].GetName())->SetPose(module_pose);
  }

  frc::SmartDashboard::PutData("Field", &m_field);

  frc::SmartDashboard::PutBoolean("Swerve/Gyro calibrating?",
                                  m_gyro.IsCalibrating());
  frc::SmartDashboard::PutNumber("Swerve/Robot heading",
                                 GetHeading().Degrees().value());
  auto wheel_speeds = each_module([](SwerveModule& m) {
    return m.GetState().speed.convert<units::meters_per_second>().value();
  });
  frc::SmartDashboard::PutNumber(
      "Robot Speed",
      std::accumulate(wheel_speeds.begin(), wheel_speeds.end(), 0.0)/4);
  frc::SmartDashboard::PutData("zeroEncodersCommand",
                               zeroEncodersCommand.get());

  for (auto &m : m_modules) m.UpdateDashboard();

  frc::SmartDashboard::PutNumber("Swerve/Gyro", m_gyro.GetAngle());

  frc::SmartDashboard::PutData("PDH", &m_pdh);

  frc::SmartDashboard::PutData("Swerve/TurnPIDController", &m_turnPID);
}

frc2::CommandPtr Drivetrain::SwerveCommand(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot) {
  return this->Run([=] {
    Drive(forward(), strafe(), rot(), false, false);
  });
}

frc2::CommandPtr Drivetrain::SwerveCommandFieldRelative(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot,
    std::function<bool()> isRed) {
  return this->Run([=] { Drive(forward(), strafe(), rot(), true, isRed()); });
}

frc2::CommandPtr Drivetrain::SwerveSlowCommand(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot,
    std::function<bool()> isRed) {
  return this->Run([=] {
    Drive(forward() / 4, strafe() / 4, rot() / 5, true, isRed());
  });
}

frc2::CommandPtr Drivetrain::DriveToPoseCommand(frc::Pose2d desiredPose,
                                      std::vector<frc::Translation2d> waypoints,
                                      units::meters_per_second_t maxSpeed,
                                      units::meters_per_second_squared_t maxAccel,
                                      units::radians_per_second_t maxAngularSpeed,
                                      units::radians_per_second_squared_t maxAngularAccel,
                                      bool isRed) {
  auto currentPose = GetPose();
  frc::TrajectoryConfig config{maxSpeed, maxAccel};
  config.SetKinematics(kDriveKinematics);
  frc::TrapezoidProfile<units::radians>::Constraints 
    kTurnConstraints{maxAngularSpeed, maxAngularAccel};

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    currentPose, waypoints, desiredPose, config);


  frc::ProfiledPIDController<units::radians> thetaController{
                                             kPTurn, kITurn, kDTurn, kTurnConstraints};
  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});
  frc2::CommandPtr m_swerveControllerCommand =
    frc2::SwerveControllerCommand<4>(trajectory,
                                     [this]() {return GetPose();}, kDriveKinematics,
                                     frc::PIDController{7.0, 0.0, 0.0},
                                     frc::PIDController{5.0, 0.0, 0.0},
                                     thetaController,
                                     [this](auto moduleStates) {SetModuleStates(moduleStates);}).ToPtr();

  return frc2::cmd::Sequence(
    std::move(m_swerveControllerCommand),
    frc2::InstantCommand(
      [this, isRed] { Drive(0_mps, 0_mps, 0_rad_per_s, true, isRed);}, {})
      .ToPtr());
}

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
  return this->RunOnce([&] { SetAbsEncoderOffset(); }).IgnoringDisable(true);
}

frc2::CommandPtr Drivetrain::CoastModeCommand(bool coast) {
  return this->StartEnd([&] { this->CoastMode(true); },
                        [&] { this->CoastMode(false); });
}

frc2::CommandPtr Drivetrain::ConfigAbsEncoderCommand() {
  return this
      ->StartEnd(
          [&] {
            CoastMode(true);
            ZeroAbsEncoders();
          },
          [&] {
            CoastMode(false);
            SetAbsEncoderOffset();
          })
      .AndThen(frc2::WaitCommand(0.5_s).ToPtr())
      .AndThen(this->RunOnce([&] { SyncEncoders(); }))
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