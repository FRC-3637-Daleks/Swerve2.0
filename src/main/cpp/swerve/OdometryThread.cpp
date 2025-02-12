#include "swerve/OdometryThread.h"

#include <ctre/phoenix6/StatusSignal.hpp>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

#include <iostream>

OdometryThread::OdometryThread(
  const std::array<SwerveModule::SignalGroup, 4> &signals,
  studica::AHRS &gyro,
  const frc::SwerveDriveKinematics<4> &kinematics,
  units::millisecond_t period
):
  m_moduleSignals{signals},
  m_gyro{gyro},
  kDriveKinematics{kinematics},
  m_odom{kDriveKinematics, GetGyroHeading(), each_position()},
  m_thread{[this, period] {Run(period);}} {
}

OdometryThread::~OdometryThread() {
  // This only matters in sim but whatever
  m_exit_flag = true;
  m_thread.join();
}

/* Lock free algorithm for single producer single consumer on-demand
  * data passing.
  * There's storage for 3 copies of the data.
  * At any given time one belongs to the consumer, one to the producer,
  * and the third one is "free". These are tracked with indecies which
  * are always mutually exclusive as managed by an atomic (lock-free).
  * 
  * Upon putting new data or fetching new data, the producer/consumer
  * swaps its index with the free index atomically, allowing it to either
  * write new data to that slot or read data that was written previously
  * to that slot.
  * 
  * The producer should produce values more often than the consumer consumes them
  * by a significant factor, but just in case the consumer reads twice
  * between 2 new values, there's also an "unreadData" flag to check if theres
  * actually new data to read
  */
void OdometryThread::RefreshData() {
  bool expected = true;
  if (m_unreadData.compare_exchange_strong(expected, false))
    m_consumerIndex = m_freeIndex.exchange(m_consumerIndex);
}

frc::Pose2d OdometryThread::GetPose() {
  return m_poses[m_consumerIndex];
}

frc::ChassisSpeeds OdometryThread::GetVel() {
  return m_vels[m_consumerIndex];
}

units::second_t OdometryThread::GetTimestamp() {
  const units::microsecond_t fpga_time{
    static_cast<double>(frc::RobotController::GetFPGATime())};
  const auto phoenix_time{ctre::phoenix6::utils::GetCurrentTime()};
  const auto offset = fpga_time - phoenix_time;
  return m_timestamps[m_consumerIndex] + offset;
}

void OdometryThread::PutData(
  const frc::Pose2d &pose, const frc::ChassisSpeeds &vel, units::second_t timestamp) {
  m_poses[m_producerIndex] = pose;
  m_vels[m_producerIndex] = vel;
  m_timestamps[m_producerIndex] = timestamp;
  m_producerIndex = m_freeIndex.exchange(m_producerIndex);
  m_unreadData = true;
}

void OdometryThread::Run(units::millisecond_t period) {
  // Configure the signal publish rates to the requested update rate
  each_module([period](SwerveModule::SignalGroup &g) {
    int retries = 4;
    while(auto ret = ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      1/period,
      g.m_drivePosition, g.m_driveVelocity,
      g.m_steerPosition, g.m_steerVelocity
    )) {
      if (retries-- == 0) {
        fmt::println(stderr, "ERROR Customizing update frequencies");
        break;
      }
    }

    return 0;
  });
  
  // Start at the origin
  m_odom.ResetPose(frc::Pose2d{});

  auto timestamp = ctre::phoenix6::utils::GetCurrentTime();

  size_t warning_count = 0;
  size_t total = 0;
  constexpr size_t n_window = 20;
  units::millisecond_t time_samples[n_window] = {0_s};
  units::millisecond_t average_time = 0_s;

  // Run indefinitely
  while (!m_exit_flag) {
    total += 1;
    // Waits for all 16 signals to come in syncronously
    // Timeout is set to the odom period
    bool timed_out = SwerveModule::SignalGroup::WaitForAllSignals(period, m_moduleSignals);

    m_odom.Update(GetGyroHeading(), each_position());

    const auto now = ctre::phoenix6::utils::GetCurrentTime();
    time_samples[total % n_window] = now - timestamp;

    if (now - timestamp - period > 0.5*period || timed_out) {
      const auto millis_since_last = units::millisecond_t{now - timestamp};
      if (warning_count++ % 20 == 0) {
        average_time = 0_s;
        for (auto t : time_samples) average_time += t;
        average_time /= n_window;

        frc::SmartDashboard::PutNumber(
          "Swerve/odom_thread/last overrun time (ms)",
          millis_since_last.value()
        );

        frc::SmartDashboard::PutNumber(
          "Swerve/odom_thread/average loop time (ms)",
          average_time.value()
        );

        frc::SmartDashboard::PutNumber(
          "Swerve/odom_thread/percent timeout",
          100.0*warning_count/total
        );
      }
    }
    
    timestamp = now;
    PutData(
      m_odom.GetPose(),
      kDriveKinematics.ToChassisSpeeds(each_state()),
      timestamp
    );
  }
}

frc::Rotation2d OdometryThread::GetGyroHeading() {
  return units::degree_t(-m_gyro.GetYaw());
}