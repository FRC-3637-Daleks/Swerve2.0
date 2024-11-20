#include "subsystems/SwerveModule.h"

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/FlywheelSim.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/time.h>

#include <iostream>
#include <numbers>
#include <random>

namespace ModuleConstants {
// Motor outputs under 4% will just be cut to 0 (brake)
constexpr double kNeutralDeadband = 0.04;

// Current Limit configs
constexpr auto kDriveMotorCurrentLimit = 60;
constexpr auto kSteerMotorCurrentLimit = 60;
// Can exceed limit for 40ms seconds
constexpr auto kCurrentLimitPeriod = 40_ms;

// Indicates time from neutral to full output
constexpr auto kRampRateSeconds = 0.2;

constexpr auto kWheelDiameter = 4_in;

constexpr double kDriveEncoderReduction = 6.75;     // reduction in drive motor
constexpr auto kDriveEncoderDistancePerRevolution = // Linear distance per revolution of motor
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;
constexpr auto kWheelMoment = .0101_kg_sq_m; //calculated based on a weight of 70lbs
constexpr auto kTalonSpeedChoreo = 5104_rpm; // choreo value
constexpr auto kTalonSpeed = 6080_rpm;       // Website value
constexpr auto kDriveAcceleration = 275_tr_per_s_sq;
constexpr auto kDistanceToRotations = kDriveEncoderDistancePerRevolution / 1_tr;

constexpr double kSteerGearReduction = 150.0 / 7.0;
constexpr auto kSteerMoment = 0.005_kg_sq_m;
constexpr auto kSteerAcceleration =
    1_tr_per_s_sq * kSteerGearReduction; // One tps^2 Acceleration, needs testing

constexpr double kDriveP = 0, kDriveI = 0, kDriveD = 0;
constexpr double kSteerP = 10, kSteerI = 0, kSteerD = 0.05;

} // namespace ModuleConstants

using namespace ModuleConstants;

class SwerveModuleSim {
public:
  SwerveModuleSim(SwerveModule &module)
      : m_driveSim(std::move(module.m_driveMotor.GetSimState())),
        m_steerSim(std::move(module.m_steerMotor.GetSimState())),
        m_encoderSim(std::move(module.m_absoluteEncoder.GetSimState())),
        m_wheelModel(frc::DCMotor::Falcon500(1),
                     kDriveEncoderReduction,
                     ModuleConstants::kWheelMoment),
        m_swivelModel(frc::DCMotor::Falcon500(1),
                     kSteerGearReduction,
                     kSteerMoment) {
    static std::random_device rng;
    std::uniform_real_distribution dist(-0.5, 0.5);

    // randomize starting positions
    m_encoderSim.SetRawPosition(units::turn_t{dist(rng)});
  }

  void update();

private:
  // hooks to hardware abstraction layer

  ctre::phoenix6::sim::TalonFXSimState m_driveSim, m_steerSim;
  ctre::phoenix6::sim::CANcoderSimState m_encoderSim;

  // tracks the simulation state for each wheel
  frc::sim::FlywheelSim m_wheelModel, m_swivelModel;
};

SwerveModule::SwerveModule(const std::string name, const int driveMotorId,
                           const int steerMotorId, const int absoluteEncoderId)
    : m_name{name}, m_driveMotor(driveMotorId, "Drivebase"), m_steerMotor(steerMotorId, "Drivebase"),
      m_absoluteEncoder(absoluteEncoderId, "Drivebase"),
      m_drivePosition(m_driveMotor.GetPosition()),
      m_driveVelocity(m_driveMotor.GetVelocity()),
      m_steerPosition(m_steerMotor.GetPosition()), //< FusedCANCoder
      m_steerVelocity(m_steerMotor.GetVelocity()),
      m_sim_state(new SwerveModuleSim(*this)) {
  
  // Reduce clutter in this function
  using namespace ctre::phoenix6;
  using namespace units;

  configs::TalonFXConfiguration steerConfig, driveConfig;

  steerConfig.WithMotorOutput(configs::MotorOutputConfigs{}
    .WithNeutralMode(signals::NeutralModeValue::Brake)
    .WithInverted(true)
  );

  driveConfig.WithMotorOutput(configs::MotorOutputConfigs{}
    .WithNeutralMode(signals::NeutralModeValue::Brake)
    .WithDutyCycleNeutralDeadband(kNeutralDeadband)
  );

  driveConfig.WithOpenLoopRamps(configs::OpenLoopRampsConfigs{}
    .WithDutyCycleOpenLoopRampPeriod(kRampRateSeconds)
    .WithVoltageOpenLoopRampPeriod(kRampRateSeconds)
    .WithTorqueOpenLoopRampPeriod(kRampRateSeconds)
  );

  driveConfig.WithClosedLoopRamps(configs::ClosedLoopRampsConfigs{}
    .WithDutyCycleClosedLoopRampPeriod(kRampRateSeconds)
    .WithVoltageClosedLoopRampPeriod(kRampRateSeconds)
    .WithTorqueClosedLoopRampPeriod(kRampRateSeconds)
  );

  // Hopefully prevents brownouts.
  driveConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
    .WithSupplyCurrentLimitEnable(true)
    .WithSupplyCurrentLimit(kDriveMotorCurrentLimit)
    .WithSupplyCurrentThreshold(kDriveMotorCurrentLimit)
    .WithSupplyTimeThreshold(kCurrentLimitPeriod.convert<seconds>().value())
  );

  steerConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
    .WithSupplyCurrentLimitEnable(true)
    .WithSupplyCurrentLimit(kSteerMotorCurrentLimit)
    .WithSupplyCurrentThreshold(kSteerMotorCurrentLimit)
    .WithSupplyTimeThreshold(kCurrentLimitPeriod.convert<seconds>().value())
  );

  // max duty cycle / corresponding velocity
  constexpr double kDriveV = 1.0 /
    (kPhysicalMaxSpeed / kDistanceToRotations).convert<tps>().value();
  driveConfig.WithSlot0(configs::Slot0Configs{}
    .WithKP(kDriveP)
    .WithKI(kDriveI)
    .WithKD(kDriveD)
    .WithKV(kDriveV)
  );

  steerConfig.WithSlot0(configs::Slot0Configs{}
    .WithKP(kSteerP)
    .WithKI(kSteerI)
    .WithKD(kSteerD)
    .WithKV(0.0)
  );

  driveConfig.WithMotionMagic(configs::MotionMagicConfigs{}
    .WithMotionMagicCruiseVelocity(kTalonSpeed.convert<turns_per_second>().value())
    .WithMotionMagicAcceleration(kDriveAcceleration.convert<turns_per_second_squared>().value())
    .WithMotionMagicJerk(0)
    .WithMotionMagicExpo_kV(kDriveV)
  );

  steerConfig.WithMotionMagic(configs::MotionMagicConfigs{}
    .WithMotionMagicCruiseVelocity(kTalonSpeed.convert<turns_per_second>().value())
    .WithMotionMagicAcceleration(kSteerAcceleration.convert<turns_per_second_squared>().value())
    .WithMotionMagicJerk(0.0)
    .WithMotionMagicExpo_kV(0.0)
    .WithMotionMagicExpo_kA(0.4)
  );

  // this object has no "With*" API for some reason
  ctre::phoenix6::configs::ClosedLoopGeneralConfigs steerClosedLoopConfig{};
  steerClosedLoopConfig.ContinuousWrap = true;
  steerConfig.WithClosedLoopGeneral(steerClosedLoopConfig);

  steerConfig.WithFeedback(configs::FeedbackConfigs{}
    .WithFeedbackSensorSource(signals::FeedbackSensorSourceValue::FusedCANcoder)
    .WithFeedbackRemoteSensorID(m_absoluteEncoder.GetDeviceID())
    .WithRotorToSensorRatio(kSteerGearReduction)
    .WithSensorToMechanismRatio(1.0)
  );

  /* Sometimes configuration fails, so we check the return code
   * and retry if needed.
   */
  int retries = 4;
  while (auto ret = m_driveMotor.GetConfigurator().Apply(driveConfig, 500_ms)) {
    if (retries-- == 0) {
      // when ret is non-zero, that means there's an error
      std::cerr << "ERROR Applying Drive Motor Configs for " << m_name
                << std::endl;
      std::cerr << "Talon ID: " << driveMotorId << ", Error: " << ret
                << std::endl;
      break;
    }
  }

  retries = 4;
  while (auto ret = m_steerMotor.GetConfigurator().Apply(steerConfig, 500_ms)) {
    if (retries-- == 0) {
      std::cerr << "ERROR Applying Steer Motor Configs for " << m_name
                << std::endl;
      std::cerr << "Talon ID: " << steerMotorId << ", Error: " << ret
                << std::endl;
      break;
    }
  }

  frc::DataLogManager::Log(
      fmt::format("Finished initializing {} swerve module", m_name));
}

SwerveModule::~SwerveModule() {}

void SwerveModule::RefreshSignals() {
  /* Refreshes all this modules signals at once.
   * This should improve performance
   */
  ctre::phoenix6::BaseStatusSignal::RefreshAll(
      m_drivePosition, m_driveVelocity, m_steerPosition, m_steerVelocity);
}

units::meter_t SwerveModule::GetModuleDistance() {
  const auto position =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_drivePosition, m_driveVelocity);
  return position * kDistanceToRotations;
}

units::meters_per_second_t SwerveModule::GetModuleVelocity() {
  return m_driveVelocity.GetValue() * kDistanceToRotations;
}

frc::Rotation2d SwerveModule::GetModuleHeading() {
  const auto position =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_steerPosition, m_steerVelocity);
  return position.convert<units::degree>();
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {GetModuleDistance(), GetModuleHeading()};
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetModuleVelocity(), GetModuleHeading()};
}

void SwerveModule::CoastMode(bool coast) {
  if (coast) {
    m_steerMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Coast);
    m_driveMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Coast);
  } else {
    m_steerMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_driveMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
  }
}

void SwerveModule::SetEncoderOffset() {
  ctre::phoenix6::configs::MagnetSensorConfigs magConfig;
  double position = m_absoluteEncoder.GetAbsolutePosition().GetValue().value();
  magConfig.WithMagnetOffset(-position);
  magConfig.WithAbsoluteSensorRange(
      ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf);
  magConfig.WithSensorDirection(
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

  m_absoluteEncoder.GetConfigurator().Apply(magConfig, 50_ms);

  SyncEncoders();
}

void SwerveModule::ZeroAbsEncoders() {
  ctre::phoenix6::configs::MagnetSensorConfigs magConfig;
  magConfig.WithMagnetOffset(0);
  magConfig.WithAbsoluteSensorRange(
      ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf);
  magConfig.WithSensorDirection(
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

  m_absoluteEncoder.GetConfigurator().Apply(magConfig, 50_ms);
}

void SwerveModule::SyncEncoders() {
  m_steerMotor.SetPosition(m_absoluteEncoder.GetAbsolutePosition().GetValue());
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState &referenceState) {
  // Optimize the reference state to prevent the module turning >90 degrees.
  auto state =
      frc::SwerveModuleState::Optimize(referenceState, GetModuleHeading());
  state.speed *= (state.angle - GetModuleHeading()).Cos();

  m_driveMotor.SetControl(
    ctre::phoenix6::controls::MotionMagicVelocityDutyCycle{
      state.speed / kDistanceToRotations}
    .WithEnableFOC(true)
    .WithSlot(0)
  );

  m_steerMotor.SetControl(
    ctre::phoenix6::controls::MotionMagicExpoDutyCycle{
      state.angle.Radians()}
    .WithEnableFOC(true)
    .WithSlot(0)
  );
}

// TODO Display things neater on the SmartDashboard.
void SwerveModule::UpdateDashboard() {
  const auto state = GetState();
  
  frc::SmartDashboard::PutNumber(
    fmt::format("Swerve/{}/heading (degrees)", m_name),
    state.angle.Degrees().value());

  frc::SmartDashboard::PutNumber(
    fmt::format("Swerve/{}/speed (mps)", m_name),
    state.speed.convert<units::mps>().value());
}

units::radian_t SwerveModule::GetAbsoluteEncoderPosition() {
  return m_absoluteEncoder.GetAbsolutePosition().GetValue();
}

// Simulation
void SwerveModule::SimulationPeriodic() {
  if (m_sim_state)
    m_sim_state->update();
}

void SwerveModuleSim::update() {
  m_driveSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_steerSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  // Simulate the wheel swiveling
  const auto prev_velocity = m_swivelModel.GetAngularVelocity();
  m_swivelModel.SetInputVoltage(m_steerSim.GetMotorVoltage());
  m_swivelModel.Update(20_ms);
  const auto average_velocity =
      (prev_velocity + m_swivelModel.GetAngularVelocity()) / 2;
  // cancoder is on mechanism and is inverted from the falcon's rotor
  m_encoderSim.AddPosition(-average_velocity * 20_ms);
  m_encoderSim.SetVelocity(-average_velocity);
  m_steerSim.AddRotorPosition(average_velocity * kSteerGearReduction * 20_ms);
  m_steerSim.SetRotorVelocity(average_velocity * kSteerGearReduction);

  // Simulate the wheel turning (ignoring changes in traction)
  m_wheelModel.SetInputVoltage(m_driveSim.GetMotorVoltage());
  m_wheelModel.Update(20_ms);

  m_driveSim.SetRotorVelocity(m_wheelModel.GetAngularVelocity() *
                              kDriveEncoderReduction);
  m_driveSim.AddRotorPosition(m_wheelModel.GetAngularVelocity() *
                              kDriveEncoderReduction * 20_ms);
}