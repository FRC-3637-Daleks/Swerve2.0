#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

// Only place constants here needed by other components
namespace KrakenModuleConstants {

// website value
constexpr auto kPhysicalMaxSpeed = 18.9_fps;

}

namespace PracticeModuleConstants {

// Values measured with the drivetrain suspended.
constexpr auto kPhysicalMaxSpeed = 15.7_fps;

}


// forward declaration
class SwerveModuleSim;

/**
 * The SwerveModule helper class consists of a steer motor and a drive motor
 * (both Falcon 500s/Krakens/Talon FXs).
 * Additionally, there is an absolute encoder (CANCoder) which regardless of
 * start position, reports the exact heading of the wheels.
 * Each module can be commanded to a certain state, that is,
 * its wheel will be driven at the specified velocity in the specified
 * direction. The Drivetrain subsystem makes use of SwerveModule objects so that
 * it doesn't need to deal with directly commanding each motor.
 * 
 */
class SwerveModule {
public:
  /**
   * The SignalGroup struct encapsulates all the signals which are used
   * at a high frequency by a SwerveModule. SignalGroup objects can be
   * copied to maintain multiple thread-safe views into the module state.
   */
  struct SignalGroup {
    using position_signal_t = ctre::phoenix6::StatusSignal<
      units::angle::turn_t>;
    using velocity_signal_t = ctre::phoenix6::StatusSignal<
      units::angular_velocity::turns_per_second_t>;

    position_signal_t m_drivePosition;
    velocity_signal_t m_driveVelocity;
    position_signal_t m_steerPosition;
    velocity_signal_t m_steerVelocity;

    // Returns the meters driven based on encoder reading.
    units::meter_t GetModuleDistance();

    // Returns the velocity of the module in m/s.
    units::meters_per_second_t GetModuleVelocity();

    // Returns the module heading in the scope [-180,180] degrees.
    frc::Rotation2d GetModuleHeading();

    // Combines GetModuleDistance() and GetModuleHeading().
    frc::SwerveModulePosition GetPosition();

    // Combines GetModuleVelocity() and GetModuleHeading().
    frc::SwerveModuleState GetState();

    // Efficiently refreshes a list of signal groups together
    template <std::same_as<SignalGroup>... T>
    static void RefreshAllSignals(T &...groups) {
      ctre::phoenix6::BaseStatusSignal::RefreshAll(
          groups.m_drivePosition..., groups.m_driveVelocity...,
          groups.m_steerPosition..., groups.m_steerVelocity...);
    }

    // Similar to RefreshAllSignals but instead waits until all signals
    // receives a fresh value from the CAN bus.
    // Blocks until specified timeout. Returns true when timeout exceeded
    // or other error
    template <std::same_as<SignalGroup>... T>
    static bool WaitForAllSignals(units::time::second_t timeout, T &...groups) {
      return ctre::phoenix6::BaseStatusSignal::WaitForAll(
        timeout,
        groups.m_drivePosition..., groups.m_driveVelocity...,
        groups.m_steerPosition..., groups.m_steerVelocity...
      ) != ctre::phoenix::StatusCode::OK;
    }

    template <auto N>
    static void RefreshAllSignals(std::array<SignalGroup, N> &groups) {
      std::apply([](auto&& ...gs) {
          RefreshAllSignals(std::forward<decltype(gs)>(gs)...);
        }, groups);
    }

    template <auto N>
    static bool WaitForAllSignals(
      units::time::second_t timeout,std::array<SignalGroup, N> &groups) {
      return std::apply([timeout](auto&& ...gs) {
          return WaitForAllSignals(timeout, std::forward<decltype(gs)>(gs)...);
        }, groups);
    }
  };
public:
  // The ctor of the SwerveModule class.
  SwerveModule(const std::string name, const int driveMotorId,
               const int steerMotorId, const int absoluteEncoderId);

  // Need to define destructor to make simulation code compile
  ~SwerveModule();

  // IMPORTANT: Need to refresh signals once per loop.
  // Getters will not return different values until signals are refreshed again
  void RefreshSignals();

  SignalGroup GetSignals() {return m_signals;}

  // This one is even more efficient than RefreshSignals as it groups ALL
  // swerve module signals into a single call
  template <std::same_as<SwerveModule>... T>
  static void RefreshAllSignals(T &...modules) {
    SignalGroup::RefreshAllSignals(modules.m_signals...);
  }
  template <auto N>
  static void RefreshAllSignals(std::array<SwerveModule, N> &modules) {
    std::apply([](auto&& ...ms) {
        RefreshAllSignals(std::forward<decltype(ms)>(ms)...);
      }, modules);
  }

  // Returns the meters driven based on encoder reading.
  units::meter_t GetModuleDistance() {
    return m_signals.GetModuleDistance();
  }

  // Returns the velocity of the module in m/s.
  units::meters_per_second_t GetModuleVelocity() {
    return m_signals.GetModuleVelocity();
  }

  // Returns the module heading in the scope [-180,180] degrees.
  frc::Rotation2d GetModuleHeading() {
    return m_signals.GetModuleHeading();
  }

  // Combines GetModuleDistance() and GetModuleHeading().
  frc::SwerveModulePosition GetPosition() {
    return m_signals.GetPosition();
  }

  // Combines GetModuleVelocity() and GetModuleHeading().
  frc::SwerveModuleState GetState() {
    return m_signals.GetState();
  }

  const std::string& GetName() {return m_name;}

  void CoastMode(bool coast);

  void SetEncoderOffset();

  void ZeroAbsEncoders();

  void SyncEncoders();

  // Commands the module to accelerate to a certain velocity and take on a
  // certain heading.
  void SetDesiredState(const frc::SwerveModuleState &state);

  // Sends the current swerve module state to the SmartDashboard.
  void UpdateDashboard();

  // Run physics simulation and update the hardware
  void SimulationPeriodic();

private:
  // Returns the absolute position of the steer motor in radians
  units::radian_t GetAbsoluteEncoderPosition();

  const std::string m_name; // Useful to identify the module.

  ctre::phoenix6::hardware::TalonFX m_driveMotor;

  ctre::phoenix6::hardware::TalonFX m_steerMotor;

  // Keeps track of the module heading between power cycles.
  ctre::phoenix6::hardware::CANcoder m_absoluteEncoder;

private: // signal object to cache
  SignalGroup m_signals;

private:
  friend class SwerveModuleSim;
  std::unique_ptr<SwerveModuleSim> m_sim_state;
};
