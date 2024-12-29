#pragma once

#include "swerve/SwerveModule.h"

#include <studica/AHRS.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>

#include <wpi/array.h>

#include <thread>
#include <atomic>
#include <utility>

/**
 * Class to manage thread syncronization for odometry thread which
 * calculates robot odometry at a higher frequency than the main thread.
 * 
 * Note, this is only really a benefit when utilizing the CANIvore AND Phoenix Pro.
 * When combined, high-throughput low-latency sensing signals can be received
 * from the CAN bus. With phoenix pro, these signals are also time syncronized.
 * This means that the sensor information from all the swerve modules
 * is received simultaneously, which is good for 2 reasons.
 * 1. the measurements themselves were taken at similar times
 * 2. the rio's CPU doesn't have to wait for some signals while others
 *    were already received and ready to be used.
 * 
 * Note: The Odometry Thread's initial pose will ALWAYS be the origin.
 * It is the responsibility of the caller to transform this pose with the
 * initial pose.
 */
class OdometryThread {
public:
  OdometryThread(
    const std::array<SwerveModule::SignalGroup, 4> &signals,
    studica::AHRS &gyro,
    const frc::SwerveDriveKinematics<4> &kinematics,
    units::millisecond_t period
  );

public:
  // Refreshes consumer cache with most recent odom data
  void RefreshData();

  // Returns the cached pose data
  frc::Pose2d GetPose();
  frc::ChassisSpeeds GetVel();
  units::second_t GetTimestamp();

private:
  std::array<SwerveModule::SignalGroup, 4> m_moduleSignals;
  studica::AHRS &m_gyro;

  frc::SwerveDriveKinematics<4> kDriveKinematics;
  frc::SwerveDriveOdometry<4> m_odom;

  std::thread m_thread;
  
  // Special case of an overwriting circular buffer
  frc::Pose2d m_poses[3];
  frc::ChassisSpeeds m_vels[3];
  units::second_t m_timestamps[3];
  uint8_t m_consumerIndex{0};
  uint8_t m_producerIndex{1};
  std::atomic<uint8_t> m_freeIndex{2};
  std::atomic<bool> m_unreadData{false};

private:
  // the actual thread function
  void Run(units::millisecond_t period);
  void PutData(const frc::Pose2d &pose, const frc::ChassisSpeeds &vel,
               units::second_t timestamp);
  frc::Rotation2d GetGyroHeading();

private:
  auto each_module(auto&& fn)
  {
    return std::apply([&fn](auto&&... ms)
    {
      return wpi::array{
        std::forward<decltype(fn)>(fn)(
          std::forward<decltype(ms)>(ms))...};
    }, m_moduleSignals);
  }

  auto each_position() {
    return each_module([](SwerveModule::SignalGroup &g) {
      return g.GetPosition();
    });
  }

  auto each_state() {
    return each_module([](SwerveModule::SignalGroup &g) {
      return g.GetState();
    });
  }
};