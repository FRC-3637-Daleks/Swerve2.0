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

#include <numeric>
#include <iostream>

namespace DriveConstants {
constexpr auto kMaxSpeed = 15.7_fps;
constexpr auto kWeight = 123_lb;
constexpr auto kMaxTurnRate = 2.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 6 * std::numbers::pi * 1_rad_per_s_sq;

constexpr auto kPeriod = 20_ms;

constexpr double kPTheta = 3.62;
constexpr double kITheta = 0.00;
constexpr double kDTheta = 0.00;

constexpr double kPXY = 5.2;
constexpr double kIXY = 0.0;
constexpr double kDXY= 0.0;

// constexpr double kPXY = 8.87; //Kc = 14.78
// constexpr double kIXY = 11.09;
// constexpr double kDXY= 1.77;


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
        m_thetaPID{kPTheta, kITheta, kDTheta, {kMaxTurnRate, kMaxTurnAcceleration}},
      m_sim_state(new DrivetrainSimulation(*this)), m_XYController(kPXY, kIXY, kDXY), 
      m_holonomicController(m_XYController, m_XYController, m_thetaPID){

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

frc::ChassisSpeeds Drivetrain::GetChassisSpeed() {
  return kDriveKinematics.ToChassisSpeeds(each_state());
}

units::meters_per_second_t Drivetrain::GetSpeed(){
  auto ret = [=]{ 
    auto vx = GetChassisSpeed().vx.to<double>();
    auto vy = GetChassisSpeed().vy.to<double>();
    auto speed = std::sqrt((pow(vx, 2))+(pow(vy, 2)));
    return (speed * 1_mps);};
  return ret();
}

bool Drivetrain::AtPose(frc::Pose2d desiredPose, frc::Pose2d tolerance) {
  auto ret = [=]{
  frc::Pose2d currentPose = GetPose();
  frc::Pose2d poseError = currentPose.RelativeTo(desiredPose);
  return (poseError.X() < tolerance.X()) &&
         (poseError.Y() < tolerance.Y()) &&
         (poseError.Rotation().Degrees() < tolerance.Rotation().Degrees()) &&
         (GetSpeed() < .02_mps) && (GetTurnRate() < 2_deg_per_s);};
    return ret();
} 

void Drivetrain::ResetOdometry(const frc::Pose2d &pose) {
  m_poseEstimator.ResetPosition(
      GetGyroHeading(), each_position(), pose);
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

  frc::SmartDashboard::PutData("Field", &m_field);

  frc::SmartDashboard::PutBoolean("Swerve/Gyro calibrating?",
                                  m_gyro.IsCalibrating());
  frc::SmartDashboard::PutNumber("Swerve/Robot heading",
                                 GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("Robot Speed (mps)",
                                 units::meters_per_second_t{GetSpeed()}.value());
  frc::SmartDashboard::PutData("zeroEncodersCommand",
                               zeroEncodersCommand.get());

  for (auto &m : m_modules) m.UpdateDashboard();

  frc::SmartDashboard::PutNumber("Swerve/Gyro", m_gyro.GetAngle());

  frc::SmartDashboard::PutData("PDH", &m_pdh);

  frc::SmartDashboard::PutData("Swerve/ThetaPIDController", &m_holonomicController.getThetaController());
  frc::SmartDashboard::PutData("Swerve/XPIDController", &m_holonomicController.getXController());
  frc::SmartDashboard::PutData("Swerve/YPIDController", &m_holonomicController.getYController());
  double error[] = {std::abs(m_holonomicController.getXController().GetPositionError()),
                    std::abs(m_holonomicController.getYController().GetPositionError()),
                    std::abs(m_holonomicController.getThetaController().GetPositionError().value())};
  frc::SmartDashboard::PutNumberArray("Error", error);
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
  return this->Run([=] {
    Drive(forward(), strafe(), rot(), true, isRed());
  });
}

frc2::CommandPtr Drivetrain::SwerveSlowCommand(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot,
     std::function<double()> throttle,
    std::function<bool()> isRed) {
  return this->Run([=] {
    auto factor = throttle()/100;
    Drive(forward() * factor, strafe() * factor, rot() * factor, true, isRed());});
  };

frc2::CommandPtr Drivetrain::DriveToPoseCommand(frc::Pose2d desiredPose,
                                      bool isRed,
                                      units::meters_per_second_t endVelo,
                                      frc::Pose2d tolerance)  {
  //I know, I know, no more commits but i had to finish what I started,
  //and I was very bored
  auto ret_cmd = RunEnd([=]  {
    GetDefaultCommand()->Cancel();
    auto currentPose = GetPose();
    auto desiredRot = desiredPose.Rotation();
    m_holonomicController.SetEnabled(true);
    m_holonomicController.SetTolerance(tolerance);
    m_field.GetObject("Desired Pose")->SetPose(desiredPose);
    auto states = m_holonomicController.Calculate(
        currentPose, desiredPose, endVelo, desiredRot);
    Drive(states.vx, states.vy, states.omega, false, isRed);},
     //sends the pose out to narnia
    [this](){m_field.GetObject("Desired Pose")->SetPose({80_m, 80_m, 0_deg});
             m_holonomicController.SetEnabled(false);})
  .Until([this, desiredPose, tolerance] {
    return AtPose(desiredPose, tolerance); 
          });
  return ret_cmd;
};

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