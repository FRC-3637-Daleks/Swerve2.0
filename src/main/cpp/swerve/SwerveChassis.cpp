#include "swerve/SwerveChassis.h"

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

namespace ChassisConstants {
constexpr auto kMaxSpeed = 15.7_fps;
constexpr auto kMaxAccel = 6_mps_sq;
constexpr auto kWeight = 70_lb;
constexpr auto kBatteryWeight = 12.9_lb;
constexpr auto kMaxTurnRate = 2.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 6 * std::numbers::pi * 1_rad_per_s_sq;

constexpr auto kPeriod = 20_ms;
constexpr auto kOdomPeriod = 5_ms;
constexpr int kOdomHertz = (1_s/kOdomPeriod).value();

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
constexpr auto kTrackWidth = 25_in; // Distance between centers of right and left wheels.
constexpr auto kWheelBase = 25_in; // Distance between centers of front and back wheels.
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

} // namespace ChassisConstants

using namespace ChassisConstants;

class SwerveChassisSimulation {
public:
    SwerveChassisSimulation(SwerveChassis& chassis)
        : m_gyroYaw(HALSIM_GetSimValueHandle(
            HALSIM_GetSimDeviceHandle("navX-Sensor[4]"), "Yaw")),
        m_poseSim(chassis.kDriveKinematics, chassis.GetHeading(),
            chassis.each_position()) {}

public:
    hal::SimDouble m_gyroYaw;
    frc::SwerveDriveOdometry<4> m_poseSim;
};

SwerveChassis::SwerveChassis()
    : kDriveKinematics{frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
                       frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
                       frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
                       frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}},
    m_modules{{{"FL", kFrontLeftDriveMotorId, kFrontLeftSteerMotorId,
                kFrontLeftAbsoluteEncoderChannel},
               {"RL", kRearLeftDriveMotorId, kRearLeftSteerMotorId,
                kRearLeftAbsoluteEncoderChannel},
               {"FR", kFrontRightDriveMotorId, kFrontRightSteerMotorId,
                kFrontRightAbsoluteEncoderChannel},
               {"RR", kRearRightDriveMotorId, kRearRightSteerMotorId,
                kRearRightAbsoluteEncoderChannel}}},
    m_gyro{studica::AHRS::NavXComType::kMXP_SPI, kOdomHertz}, // update rate
    m_odom_thread{each_module([](SwerveModule& m) { return m.GetSignals(); }),
                  m_gyro, kDriveKinematics, kOdomPeriod},
    m_holonomicController(kTranslatePID, kTranslatePID, kThetaPID),
    m_sim_state(new SwerveChassisSimulation(*this)) {

    InitializeDashboard();

    frc::DataLogManager::Log(
        fmt::format("Finished initializing SwerveChassis subsystem."));
}

frc::Pose2d SwerveChassis::GetSimulatedGroundTruth() {
    return m_sim_state->m_poseSim.GetPose();
}

void SwerveChassis::Periodic() {
    // Do this once per loop
    SwerveModule::RefreshAllSignals(m_modules);

    // Polls the odom thread for most up to date pose as if this loop iteration
    m_odom_thread.RefreshData();

    this->UpdateDashboard();
}

SwerveChassis::~SwerveChassis() {}

void SwerveChassis::RobotRelativeDrive(const frc::ChassisSpeeds& cmd_vel) {
    auto states = kDriveKinematics.ToSwerveModuleStates(cmd_vel);

    // Occasionally a drive motor is commanded to go faster than its maximum
    // output can sustain. Desaturation lowers the module speeds so that no
    // motor is driven above its maximum speed, while preserving the intended
    // motion.
    kDriveKinematics.DesaturateWheelSpeeds(
        &states, ModuleConstants::kPhysicalMaxSpeed);

    // Finally each of the desired states can be sent as commands to the
    // modules.
    SetModuleStates(states);
}

void SwerveChassis::Drive(const frc::ChassisSpeeds& cmd_vel) {
    RobotRelativeDrive(
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(cmd_vel, GetHeading()));
}

void SwerveChassis::SetModuleStates(
    const wpi::array<frc::SwerveModuleState, kNumModules>& desiredStates) {
    for (int i = 0; i < kNumModules; i++)
        m_modules[i].SetDesiredState(desiredStates[i]);
}

frc::Rotation2d SwerveChassis::GetHeading() { return GetPose().Rotation(); }

frc::Rotation2d SwerveChassis::GetGyroHeading() {
    // The yaw for Nav-X doesn't use conventional rotation
    return units::degree_t(-m_gyro.GetYaw());
}

void SwerveChassis::ZeroHeading() {
    auto pose = GetPose();
    ResetOdometry(frc::Pose2d{pose.X(), pose.Y(), 0_deg});
}

void SwerveChassis::SyncEncoders() {
    for (auto& m : m_modules)
        m.SyncEncoders();
}

void SwerveChassis::CoastMode(bool coast) {
    for (auto& m : m_modules)
        m.CoastMode(coast);
}

void SwerveChassis::DriveToPose(const frc::Pose2d& desiredPose,
    frc::ChassisSpeeds feedForward,
    const frc::Pose2d& tolerance) {
    auto currentPose = GetPose();
    auto endVelo = units::math::sqrt(units::math::pow<2>(feedForward.vx) +
        units::math::pow<2>(feedForward.vy));
    auto rot = units::math::atan2(feedForward.vy, feedForward.vx);
    frc::Pose2d newPose = {desiredPose.X(), desiredPose.Y(), rot};
    m_holonomicController.SetTolerance(tolerance);
    m_field.GetObject("Desired Pose")->SetPose(desiredPose);
    auto speeds = m_holonomicController.Calculate(currentPose, newPose, endVelo,
        desiredPose.Rotation());
    frc::ChassisSpeeds finalSpeeds = {speeds.vx, speeds.vy,
                                      (speeds.omega + feedForward.omega)};
    RobotRelativeDrive(finalSpeeds);
}

units::degrees_per_second_t SwerveChassis::GetTurnRate() {
    return -m_gyro.GetRate() * 1_deg_per_s;
}

frc::Pose2d SwerveChassis::GetOdomPose() {
    constexpr frc::Pose2d origin{};
    const auto odom_transform = m_odom_thread.GetPose() - origin;
    return origin + m_initial_transform + odom_transform; // order matters
}

units::second_t SwerveChassis::GetOdomTimestamp() {
    return m_odom_thread.GetTimestamp();
}

frc::Pose2d SwerveChassis::GetPose() {
    constexpr frc::Pose2d origin{};
    const auto odom_transform = GetOdomPose() - origin;
    return origin + m_map_to_odom + odom_transform; // order matters
}

frc::ChassisSpeeds SwerveChassis::GetChassisSpeed() {
    return m_odom_thread.GetVel();
}

units::meters_per_second_t SwerveChassis::GetSpeed() {
    const auto speeds = GetChassisSpeed();
    return units::math::sqrt(units::math::pow<2>(speeds.vx) +
        units::math::pow<2>(speeds.vy));
}

bool SwerveChassis::AtPose(const frc::Pose2d& desiredPose,
    const frc::Pose2d& tolerance) {
    auto currentPose = GetPose();
    auto poseError = currentPose.RelativeTo(desiredPose);
    return (units::math::abs(poseError.X()) < tolerance.X()) &&
        (units::math::abs(poseError.Y()) < tolerance.Y()) &&
        (units::math::abs(poseError.Rotation().Degrees()) <
            tolerance.Rotation().Degrees());
}

bool SwerveChassis::IsStopped() {
    auto currentSpeed = GetSpeed();
    return currentSpeed < 0.1_mps;
}

void SwerveChassis::ResetOdometry(const frc::Pose2d& pose) {
    // m_initial_transform is a "hardcoded" offset.
    // it is defined to be the transform which when added to the other
    // components of GetPose(), will produce the 'pose' passed in here.
    // m_map_to_odom is reset since this value is effectively invalidated
    // by resetting the odometry.
    constexpr frc::Pose2d origin{};
    m_initial_transform = {};
    m_map_to_odom = {};
    const auto odom_transform = GetOdomPose() - origin;
    m_initial_transform = (pose + odom_transform.Inverse()) - origin;
}

void SwerveChassis::SetMapToOdom(const frc::Transform2d& transform) {
    m_map_to_odom = transform;
}

void SwerveChassis::InitializeDashboard() {
    // PutData is persistent, these objects only need to be passed in once
    frc::SmartDashboard::PutData("Field", &m_field);
    m_field.GetObject("reset_point")->SetPose(frc::Pose2d{});
    frc::SmartDashboard::PutData("gyro", &m_gyro);

    frc::SmartDashboard::PutData("Swerve/ThetaPIDController",
        &m_holonomicController.GetThetaController());
    frc::SmartDashboard::PutData("Swerve/XPIDController",
        &m_holonomicController.GetXController());
    frc::SmartDashboard::PutData("Swerve/YPIDController",
        &m_holonomicController.GetYController());
}

void SwerveChassis::UpdateDashboard() {
    const auto robot_center = this->GetPose();

    m_field.SetRobotPose(this->GetPose());
    m_field.GetObject("init pose")
        ->SetPose(frc::Pose2d{}.TransformBy(m_initial_transform));

    constexpr int xs[] = {1, 1, -1, -1};
    constexpr int ys[] = {1, -1, 1, -1};
    for (int i = 0; i < kNumModules; i++) {
        const auto module_pose = robot_center.TransformBy(
            {xs[i] * kWheelBase / 2, ys[i] * kTrackWidth / 2,
             m_modules[i].GetState().angle});
        m_field.GetObject(m_modules[i].GetName())->SetPose(module_pose);
    }

    frc::SmartDashboard::PutBoolean("Swerve/Gyro calibrating?",
        m_gyro.IsCalibrating());
    frc::SmartDashboard::PutNumber("Swerve/Robot heading",
        GetHeading().Degrees().value());
    frc::SmartDashboard::PutNumber(
        "Robot Speed (mps)", units::meters_per_second_t{GetSpeed()}.value());

    for (auto& m : m_modules)
        m.UpdateDashboard();

    frc::SmartDashboard::PutNumber("Swerve/Gyro", m_gyro.GetAngle());

    double error[] = {
        m_holonomicController.GetXController().GetError(),
        m_holonomicController.GetYController().GetError(),
        m_holonomicController.GetThetaController().GetPositionError().value()};
    frc::SmartDashboard::PutNumberArray("Error", error);
}

void SwerveChassis::SimulationPeriodic() {
    if (!m_sim_state)
        return;

    for (auto& m : m_modules)
        m.SimulationPeriodic();

    // Assume perfect kinematics and get the new gyro angle
    const auto chassis_speed = kDriveKinematics.ToChassisSpeeds(each_state());
    const auto theta = m_sim_state->m_poseSim.GetPose().Rotation();
    const auto new_theta =
        theta.RotateBy(units::radian_t{chassis_speed.omega * 20_ms});
    // robot nav x defines clockwise as positive instead of counterclockwise
    m_sim_state->m_gyroYaw.Set(-new_theta.Degrees().value());

    // Feed this simulated gyro angle into the odometry to get simulated
    // position
    m_sim_state->m_poseSim.Update(new_theta, each_position());

    m_field.GetObject("simulation")->SetPose(m_sim_state->m_poseSim.GetPose());
}