#include "subsystems/Drivetrain.h"

#include <cmath> // Make sure to include cmath for std::fmod

#include <frc/DataLogManager.h>
#include <frc/I2C.h>
#include <frc/SPI.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/WaitCommand.h>
#include <wpi/array.h>

#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

namespace DriveConstants {
constexpr auto kMaxSpeed = 15.7_fps;
constexpr auto kWeight = 123_lb;
constexpr auto kMaxTurnRate = 2.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 6 * std::numbers::pi * 1_rad_per_s_sq;

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
                  {drivetrain.m_frontLeft.GetPosition(),
                   drivetrain.m_frontRight.GetPosition(),
                   drivetrain.m_rearLeft.GetPosition(),
                   drivetrain.m_rearRight.GetPosition()}) {}

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
      m_frontLeft{"FL",
        kFrontLeftDriveMotorId,
        kFrontLeftSteerMotorId,
        kFrontLeftAbsoluteEncoderChannel},
      m_rearLeft{"RL",
        kRearLeftDriveMotorId,
        kRearLeftSteerMotorId,
        kRearLeftAbsoluteEncoderChannel},
      m_frontRight{"FR",
        kFrontRightDriveMotorId,
        kFrontRightSteerMotorId,
        kFrontRightAbsoluteEncoderChannel},
      m_rearRight{"RR",
        kRearRightDriveMotorId,
        kRearRightSteerMotorId,
        kRearRightAbsoluteEncoderChannel},
      m_gyro{frc::SPI::Port::kMXP},
      m_pdh{kPDH, frc::PowerDistribution::ModuleType::kRev},
      m_poseEstimator{kDriveKinematics, GetGyroHeading(),
        wpi::array<frc::SwerveModulePosition, 4U>{
          m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
          m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
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
  SwerveModule::RefreshAllSignals(m_frontLeft, m_frontRight,
                                  m_rearLeft, m_rearRight);

  // Update the odometry with the current gyro angle and module states.
  auto fl_pos = m_frontLeft.GetPosition();
  auto fr_pos = m_frontRight.GetPosition();
  auto rl_pos = m_rearLeft.GetPosition();
  auto rr_pos = m_rearRight.GetPosition();

  m_poseEstimator.Update(GetGyroHeading(), {fl_pos, fr_pos, rl_pos, rr_pos});

  this->UpdateDashboard();
}

Drivetrain::~Drivetrain() {}

void Drivetrain::Drive(units::meters_per_second_t forwardSpeed,
                       units::meters_per_second_t strafeSpeed,
                       units::radians_per_second_t angularSpeed,
                       bool fieldRelative, bool isRed) {
  //  Use the kinematics model to get from the set of commanded speeds to a set
  //  of states that can be commanded to each module.
  auto states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});

  if (fieldRelative) {
    if (isRed)
      states = kDriveKinematics.ToSwerveModuleStates(
          frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              -1 * forwardSpeed, -1 * strafeSpeed, angularSpeed, GetHeading()));
    else
      states = kDriveKinematics.ToSwerveModuleStates(
          frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              forwardSpeed, strafeSpeed, angularSpeed, GetHeading()));
  } else {
    states = kDriveKinematics.ToSwerveModuleStates(
        frc::ChassisSpeeds{forwardSpeed, strafeSpeed, angularSpeed});
  }

  // Occasionally a drive motor is commanded to go faster than its maximum
  // output can sustain. Desaturation lowers the module speeds so that no motor
  // is driven above its maximum speed, while preserving the intended motion.
  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  // Finally each of the desired states can be sent as commands to the modules.
  auto [fl, fr, rl, rr] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(rl);
  m_rearRight.SetDesiredState(rr);
}

void Drivetrain::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

frc::Rotation2d Drivetrain::GetHeading() { return GetPose().Rotation(); }

frc::Rotation2d Drivetrain::GetGyroHeading() {
  return units::degree_t(-m_gyro.GetYaw());
}

// void Drivetrain::ZeroHeading() { m_gyro.Reset();}
void Drivetrain::ZeroHeading() {
  auto pose = GetPose();
  ResetOdometry(frc::Pose2d{pose.X(), pose.Y(), 0_deg});
}

void Drivetrain::ZeroAbsEncoders() {
  m_frontLeft.ZeroAbsEncoders();
  m_frontRight.ZeroAbsEncoders();
  m_rearLeft.ZeroAbsEncoders();
  m_rearRight.ZeroAbsEncoders();
}

void Drivetrain::SetAbsEncoderOffset() {
  m_frontLeft.SetEncoderOffset();
  m_frontRight.SetEncoderOffset();
  m_rearLeft.SetEncoderOffset();
  m_rearRight.SetEncoderOffset();
}

void Drivetrain::SyncEncoders() {
  m_frontLeft.SyncEncoders();
  m_frontRight.SyncEncoders();
  m_rearLeft.SyncEncoders();
  m_rearRight.SyncEncoders();
}

void Drivetrain::CoastMode(bool coast) {
  m_frontLeft.CoastMode(coast);
  m_frontRight.CoastMode(coast);
  m_rearLeft.CoastMode(coast);
  m_rearRight.CoastMode(coast);
}
units::degrees_per_second_t Drivetrain::GetTurnRate() {
  return -m_gyro.GetRate() * 1_deg_per_s;
}

frc::Pose2d Drivetrain::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

frc::ChassisSpeeds Drivetrain::GetSpeed() {
  return kDriveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(), m_frontRight.GetState(),
      m_rearLeft.GetState(), m_rearRight.GetState());
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose) {
  m_poseEstimator.ResetPosition(
      GetGyroHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

void Drivetrain::UpdateDashboard() {
  const auto robot_center = this->GetPose();
  m_field.SetRobotPose(this->GetPose());

  const auto fl_pose = robot_center.TransformBy(
      {kWheelBase / 2, kTrackWidth / 2, m_frontLeft.GetState().angle});
  m_field.GetObject("FL")->SetPose(fl_pose);

  const auto fr_pose = robot_center.TransformBy(
      {kWheelBase / 2, -kTrackWidth / 2, m_frontRight.GetState().angle});
  m_field.GetObject("FR")->SetPose(fr_pose);

  const auto rl_pose = robot_center.TransformBy(
      {-kWheelBase / 2, kTrackWidth / 2, m_rearLeft.GetState().angle});
  m_field.GetObject("RL")->SetPose(rl_pose);

  const auto rr_pose = robot_center.TransformBy(
      {-kWheelBase / 2, -kTrackWidth / 2, m_rearRight.GetState().angle});
  m_field.GetObject("RR")->SetPose(rr_pose);

  frc::SmartDashboard::PutData("Field", &m_field);

  frc::SmartDashboard::PutBoolean("Swerve/Gyro calibrating?",
                                  m_gyro.IsCalibrating());
  frc::SmartDashboard::PutNumber("Swerve/Robot heading",
                                 GetHeading().Degrees().value());
  double swerveStates[] = {m_frontLeft.GetState().angle.Radians().value(),
                           m_frontLeft.GetState().speed.value(),
                           m_frontRight.GetState().angle.Radians().value(),
                           m_frontRight.GetState().speed.value(),
                           m_rearLeft.GetState().angle.Radians().value(),
                           m_rearLeft.GetState().speed.value(),
                           m_rearRight.GetState().angle.Radians().value(),
                           m_rearRight.GetState().speed.value()};
  frc::SmartDashboard::PutNumberArray(
      "Swerve/Swerve Module States",
      swerveStates); // Have to initialize array separately due as an error
                     // occurs when an array attempts to initialize as a
                     // parameter.
  frc::SmartDashboard::PutNumber(
      "Robot Speed",
      (swerveStates[1] + swerveStates[3] + swerveStates[5] + swerveStates[7]) /
          4);
  frc::SmartDashboard::PutData("zeroEncodersCommand",
                               zeroEncodersCommand.get());
  m_frontLeft.UpdateDashboard();
  m_rearLeft.UpdateDashboard();
  m_frontRight.UpdateDashboard();
  m_rearRight.UpdateDashboard();

  frc::SmartDashboard::PutNumber("Swerve/Gyro", m_gyro.GetAngle());

  frc::SmartDashboard::PutData("PDH", &m_pdh);

  frc::SmartDashboard::PutData("Swerve/TurnPIDController", &m_turnPID);

  // frc::SmartDashboard::PutNumber("PDH/Voltage", m_pdh.GetVoltage());

  // frc::SmartDashboard::PutNumber("PDH/Total Current",
  // m_pdh.GetTotalCurrent());
}

frc2::CommandPtr Drivetrain::SwerveCommand(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot) {
  // fmt::print("making command\n");
  return this->Run([=] {
    // fmt::print("starting drive command\n");
    Drive(forward(), strafe(), rot(), false, false);
    // fmt::print("sent drive command\n");
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
  // fmt::print("making command\n");
  return this->Run([=] {
    // fmt::print("starting drive command\n");
    Drive(forward() / 4, strafe() / 4, rot() / 5, true, isRed());
    // fmt::print("sent drive command\n");
  });
}

frc2::CommandPtr Drivetrain::ZeroHeadingCommand() {
  return this->RunOnce([&] { ZeroHeading(); });
}

frc2::CommandPtr Drivetrain::ZeroAbsEncodersCommand() {
  return this
      ->RunOnce([&] {
        fmt::print("inside ZeroAbsEncodersCommand");
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

  m_frontLeft.SimulationPeriodic();
  m_rearLeft.SimulationPeriodic();
  m_frontRight.SimulationPeriodic();
  m_rearRight.SimulationPeriodic();

  // Assume perfect kinematics and get the new gyro angle
  const auto chassis_speed = kDriveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(),
      m_rearRight.GetState());

  const auto theta = m_sim_state->m_poseSim.GetPose().Rotation();
  const auto new_theta =
      theta.RotateBy(units::radian_t{chassis_speed.omega * 20_ms});
  // robot nav x defines clockwise as positive instead of counterclockwise
  m_sim_state->m_gyroYaw.Set(-new_theta.Degrees().value());

  // Feed this simulated gyro angle into the odometry to get simulated position
  auto fl_pos = m_frontLeft.GetPosition();
  auto fr_pos = m_frontRight.GetPosition();
  auto rl_pos = m_rearLeft.GetPosition();
  auto rr_pos = m_rearRight.GetPosition();

  // Modify this to simulate different kinds of odom error
  fl_pos.angle = fl_pos.angle.Degrees();
  fr_pos.angle = fr_pos.angle.Degrees();
  rl_pos.angle = rl_pos.angle.Degrees();
  rr_pos.angle = rr_pos.angle.Degrees();

  m_sim_state->m_poseSim.Update(new_theta, {fl_pos, fr_pos, rl_pos, rr_pos});

  m_field.GetObject("simulation")->SetPose(m_sim_state->m_poseSim.GetPose());
}