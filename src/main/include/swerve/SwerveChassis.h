#pragma once

#include <studica/AHRS.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/estimator/UnscentedKalmanFilter.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include <memory>
#include <numbers>
#include <utility>

#include "DrivetrainInterfaceHelper.h"
#include "swerve/OdometryThread.h"
#include "swerve/SwerveModule.h"

// Forward Declaration
class SwerveChassisSimulation;

/**
 * The SwerveChassis subsystem contains four swerve modules and a gyroscope. The
 * SwerveChassis can be driven at a certain speed represented as a 2-dimensional
 * translation on the field. This translation can be commanded either relative
 * to the front of the robot, or relative to a certain absolute position. These
 * two modes are called "robot relative" and "field relative" respectively. The
 * SwerveChassis can also be commanded to rotate at a given speed independent of
 * its translation.
 */
class SwerveChassis: public frc2::SubsystemBase {
private:
    enum module_id {
        kFrontLeft = 0,
        kFrontRight,
        kRearLeft,
        kRearRight,
        kNumModules
    };

public:
    using module_states_t = wpi::array<frc::SwerveModuleState, kNumModules>;

public:
    // The ctor of the Drivetrain subsystem.
    SwerveChassis();

    // Need to define destructor to make simulation code compile
    ~SwerveChassis();

    // Updates the odometer and SmartDashboard.
    void Periodic() override;

    // Executes the simulation
    void SimulationPeriodic() override;

    // Executes given command velocity (x, y, omega)
    // Motion is relative to the robot's frame
    // This is useful when a driver is looking through a camera
    void RobotRelativeDrive(const frc::ChassisSpeeds& cmd_vel);

    // Executes given command velocity (x, y, omega)
    // X and Y velocities are relative to the field coordinates.
    void Drive(const frc::ChassisSpeeds& cmd_vel);

    // Sets the state of each swerve module.
    void SetModuleStates(const module_states_t& desiredStates);

    // Returns the heading of the robot.
    frc::Rotation2d GetHeading();

    frc::Rotation2d GetGyroHeading();

    // Zeroes the robot heading.
    void ZeroHeading();

    void SyncEncoders();

    void CoastMode(bool coast);

    void DriveToPose(const frc::Pose2d& desiredPose,
        frc::ChassisSpeeds feedForward = {0_mps, 0_mps, 0_rpm},
        const frc::Pose2d& tolerance = {0.06_m, 0.06_m, 3_deg});

    // Returns the rotational velocity of the robot in degrees per second.
    units::degrees_per_second_t GetTurnRate();

    // Returns the uncorrected odometry transform for streaming to ROS
    frc::Pose2d GetOdomPose();

    // Returns the timestamp associated with the current odometry pose
    units::second_t GetOdomTimestamp();

    // Returns the robot heading and translation as a Pose2d.
    frc::Pose2d GetPose();

    frc::Pose2d GetSimulatedGroundTruth();

    // Returns Current Chassis Speed
    frc::ChassisSpeeds GetChassisSpeed();

    units::meters_per_second_t GetSpeed();

    bool AtPose(const frc::Pose2d& desiredPose,
        const frc::Pose2d& tolerance = {0.06_m, 0.06_m, 2_deg});

    bool IsStopped();

    void ResetOdometry(const frc::Pose2d& pose);

    void SetMapToOdom(const frc::Transform2d& transform);

    // Display useful information on Shuffleboard.
    void InitializeDashboard();
    void UpdateDashboard();
    frc::Field2d& GetField() { return m_field; }

private:
    frc::SwerveDriveKinematics<4> kDriveKinematics;
    std::array<SwerveModule, kNumModules> m_modules;

    studica::AHRS m_gyro;

    OdometryThread m_odom_thread;
    frc::Transform2d m_initial_transform; //< initial pose if known
    frc::Transform2d m_map_to_odom;       //< pose correction from sensors

    // Field widget for Shuffleboard.
    frc::Field2d m_field;

    // Stores controllers for each motion axis
    frc::HolonomicDriveController m_holonomicController;

private:
    friend class SwerveChassisSimulation;
    std::unique_ptr<SwerveChassisSimulation> m_sim_state;

protected:
    // magic to make doing stuff for every module easier
    auto each_module(auto&& fn) {
        return std::apply(
            [&fn](auto &&...ms) {
                return wpi::array{std::forward<decltype(fn)>(fn)(
                    std::forward<decltype(ms)>(ms))...};
            },
            m_modules);
    }

    auto each_position() {
        return each_module([](SwerveModule& m) { return m.GetPosition(); });
    }

    auto each_state() {
        return each_module([](SwerveModule& m) { return m.GetState(); });
    }
};
