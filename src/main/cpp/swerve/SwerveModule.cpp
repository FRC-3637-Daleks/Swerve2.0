#include "swerve/SwerveModule.h"

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/LinearFilter.h>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

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
#include <units/angle.h>

#include <iostream>
#include <numbers>
#include <random>
namespace KrakenModuleConstants {
  // Motor outputs under 4% will just be cut to 0 (brake)
constexpr double kNeutralDeadband = 0.04;

// Current Limit configs
constexpr auto kDriveMotorCurrentLimit = 70_A;
constexpr auto kSteerMotorCurrentLimit = 30_A;
// Can exceed limit for 40ms seconds
constexpr auto kCurrentLimitPeriod = 40_ms;

// Indicates time from neutral to full output
constexpr auto kRampRate = 0.2_s;

constexpr auto kWheelDiameter = 4_in;

constexpr double kDriveEncoderReduction = 5.36;     // reduction in drive motor
constexpr auto kDriveEncoderDistancePerRevolution = // Linear distance per revolution of motor
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;
constexpr auto kWheelMoment = .0101_kg_sq_m; //calculated based on a weight of 70lbs
constexpr auto kMotorSpeed = 5800_rpm;       // Website value
constexpr auto kDriveMaxAcceleration = 500_tr_per_s_sq;
constexpr auto kDriveTargetAcceleration = 300_tr_per_s_sq;
constexpr auto kDistanceToRotations = kDriveEncoderDistancePerRevolution / 1_tr;

constexpr double kSteerGearReduction = 12.8;
constexpr auto kSteerMoment = 0.005_kg_sq_m;
constexpr auto kSteerAcceleration = 135.7_tr_per_s_sq * 2; //Measured empirically, rough guess
constexpr auto kSteerSpeed = kMotorSpeed / kSteerGearReduction;

constexpr double kDriveP = 0, kDriveI = 0.1, kDriveD = 0;
constexpr double kSteerP = 10, kSteerI = 0, kSteerD = 0.002, kSteerS = 0.03;

const auto MotorModel = [] (int N=1) {return frc::DCMotor::KrakenX60FOC(N);};

}

namespace PracticeModuleConstants {
// Motor outputs under 4% will just be cut to 0 (brake)
constexpr double kNeutralDeadband = 0.04;

// Current Limit configs
constexpr auto kDriveMotorCurrentLimit = 70_A;
constexpr auto kSteerMotorCurrentLimit = 30_A;
// Can exceed limit for 40ms seconds
constexpr auto kCurrentLimitPeriod = 40_ms;

// Indicates time from neutral to full output
constexpr auto kRampRate = 0.2_s;

constexpr auto kWheelDiameter = 4_in;

constexpr double kDriveEncoderReduction = 6.75;     // reduction in drive motor
constexpr auto kDriveEncoderDistancePerRevolution = // Linear distance per revolution of motor
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;
constexpr auto kWheelMoment = .0101_kg_sq_m; //calculated based on a weight of 70lbs
constexpr auto kMotorSpeedChoreo = 5104_rpm; // choreo value
constexpr auto kMotorSpeed = 6080_rpm;       // Website value
constexpr auto kDriveMaxAcceleration = 500_tr_per_s_sq;
constexpr auto kDriveTargetAcceleration = 300_tr_per_s_sq;
constexpr auto kDistanceToRotations = kDriveEncoderDistancePerRevolution / 1_tr;

constexpr double kSteerGearReduction = 150.0 / 7.0;
constexpr auto kSteerMoment = 0.0001_kg_sq_m;  // Reduced to near 0-mass for smooth sim driving
constexpr auto kSteerAcceleration = 135.7_tr_per_s_sq * 2; //Measured empirically, rough guess
constexpr auto kSteerSpeed = kMotorSpeed / kSteerGearReduction;

constexpr double kDriveP = 0, kDriveI = 0.1, kDriveD = 0;
constexpr double kSteerP = 10, kSteerI = 0, kSteerD = 0.02, kSteerS = 0.03;

const auto MotorModel = [] (int N=1) {return frc::DCMotor::Falcon500FOC(N);};

} // namespace PracticeModuleConstants

using namespace PracticeModuleConstants;

class SwerveModuleSim {
public:
  SwerveModuleSim(SwerveModule &module)
      : m_driveSim(std::move(module.m_driveMotor.GetSimState())),
        m_steerSim(std::move(module.m_steerMotor.GetSimState())),
        m_encoderSim(std::move(module.m_absoluteEncoder.GetSimState())),
        m_wheelModel(
          frc::LinearSystemId::DCMotorSystem(
            MotorModel(),
            kWheelMoment,
            kDriveEncoderReduction),
          MotorModel()
        ),
        m_swivelModel(
          frc::LinearSystemId::DCMotorSystem(
            MotorModel(),
            kSteerMoment,
            kSteerGearReduction),
          MotorModel()
        ) {
    static std::random_device rng;
    std::uniform_real_distribution dist(-0.5, 0.5);

    // randomize starting positions
    m_swivelModel.SetState(dist(rng)*1_tr, 0_rpm);
  }

  void update();

private:
  // hooks to hardware abstraction layer

  ctre::phoenix6::sim::TalonFXSimState m_driveSim, m_steerSim;
  ctre::phoenix6::sim::CANcoderSimState m_encoderSim;

  // tracks the simulation state for each wheel
  frc::sim::DCMotorSim m_wheelModel, m_swivelModel;
};

SwerveModule::SwerveModule(const std::string name, const int driveMotorId,
                           const int steerMotorId, const int absoluteEncoderId)
    : m_name{name}, m_driveMotor(driveMotorId, "Drivebase"), m_steerMotor(steerMotorId, "Drivebase"),
      m_absoluteEncoder(absoluteEncoderId, "Drivebase"),
      m_signals{
        m_driveMotor.GetPosition(),
        m_driveMotor.GetVelocity(),
        m_steerMotor.GetPosition(), //< FusedCANCoder
        m_steerMotor.GetVelocity()
      },
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
    .WithDutyCycleOpenLoopRampPeriod(kRampRate)
    .WithVoltageOpenLoopRampPeriod(kRampRate)
    .WithTorqueOpenLoopRampPeriod(kRampRate)
  );

  driveConfig.WithClosedLoopRamps(configs::ClosedLoopRampsConfigs{}
    .WithDutyCycleClosedLoopRampPeriod(kRampRate)
    .WithVoltageClosedLoopRampPeriod(kRampRate)
    .WithTorqueClosedLoopRampPeriod(kRampRate)
  );

  // CTRE Alleges that the new firmware has sensible default current limits
  // driveConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
  //   .WithSupplyCurrentLimitEnable(true)
  //   .WithSupplyCurrentLimit(kDriveMotorCurrentLimit)
  // );

  // steerConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
  //   .WithSupplyCurrentLimitEnable(true)
  //   .WithSupplyCurrentLimit(kSteerMotorCurrentLimit)
  // );

  // max duty cycle / corresponding velocity
  constexpr auto kDriveV = 1.0/(kPhysicalMaxSpeed/kDistanceToRotations);
  //constexpr auto kDriveA = 1.0/units::turns_per_second_squared_t{kDriveMaxAcceleration};
  driveConfig.WithSlot0(configs::Slot0Configs{}
    .WithKP(kDriveP)
    .WithKI(kDriveI)
    .WithKD(kDriveD)
    .WithKV(ctre::unit::scalar_per_turn_per_second_t{kDriveV}.value())
  );
  
  constexpr auto kSteerV = 1.0/units::turns_per_second_t{kSteerSpeed};
  constexpr auto kSteerA = 1.0/units::turns_per_second_squared_t{kSteerAcceleration};
  steerConfig.WithSlot0(configs::Slot0Configs{}
    .WithKP(kSteerP)
    .WithKI(kSteerI)
    .WithKD(kSteerD)
    .WithKV(kSteerV.value())
    .WithKA(kSteerA.value())
    .WithKS(kSteerS)
  );

  driveConfig.WithMotionMagic(configs::MotionMagicConfigs{}
    .WithMotionMagicAcceleration(kDriveTargetAcceleration)
  );

  steerConfig.WithMotionMagic(configs::MotionMagicConfigs{}
    .WithMotionMagicCruiseVelocity(kSteerSpeed)
    .WithMotionMagicAcceleration(kSteerAcceleration)
    .WithMotionMagicExpo_kV(12.0_V*kSteerV)
    .WithMotionMagicExpo_kA(12.0_V*kSteerA)
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
      m_signals.m_drivePosition, m_signals.m_driveVelocity,
      m_signals.m_steerPosition, m_signals.m_steerVelocity);
}

units::meter_t SwerveModule::SignalGroup::GetModuleDistance() {
  const auto position =
    ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      m_drivePosition, m_driveVelocity);
  return position * kDistanceToRotations;
}

units::meters_per_second_t SwerveModule::SignalGroup::GetModuleVelocity() {
  return m_driveVelocity.GetValue() * kDistanceToRotations;
}

frc::Rotation2d SwerveModule::SignalGroup::GetModuleHeading() {
  const auto position =
    ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      m_steerPosition, m_steerVelocity);
  return position.convert<units::degree>();
}

frc::SwerveModulePosition SwerveModule::SignalGroup::GetPosition() {
  return {GetModuleDistance(), GetModuleHeading()};
}

frc::SwerveModuleState SwerveModule::SignalGroup::GetState() {
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
  auto position = m_absoluteEncoder.GetAbsolutePosition().GetValue();
  magConfig.WithMagnetOffset(-position);
  magConfig.WithAbsoluteSensorDiscontinuityPoint(0.5_tr);
  magConfig.WithSensorDirection(
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

  m_absoluteEncoder.GetConfigurator().Apply(magConfig, 50_ms);

  SyncEncoders();
}

void SwerveModule::ZeroAbsEncoders() {
  ctre::phoenix6::configs::MagnetSensorConfigs magConfig;
  magConfig.WithMagnetOffset(0_tr);
  magConfig.WithAbsoluteSensorDiscontinuityPoint(0.5_tr);
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
  auto state = referenceState;
  state.Optimize(GetModuleHeading());
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
  m_encoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  // Simulate the wheel swiveling
  m_swivelModel.SetInputVoltage(m_steerSim.GetMotorVoltage());
  m_swivelModel.Update(20_ms);
  // cancoder is on mechanism and is inverted from the falcon's rotor
  m_encoderSim.SetRawPosition(-m_swivelModel.GetAngularPosition());
  m_encoderSim.SetVelocity(-m_swivelModel.GetAngularVelocity());
  m_steerSim.SetRawRotorPosition(m_swivelModel.GetAngularPosition() * kSteerGearReduction);
  m_steerSim.SetRotorVelocity(m_swivelModel.GetAngularVelocity() * kSteerGearReduction);
  m_steerSim.SetRotorAcceleration(m_swivelModel.GetAngularAcceleration() * kSteerGearReduction);

  // Simulate the wheel turning (ignoring changes in traction)
  m_wheelModel.SetInputVoltage(m_driveSim.GetMotorVoltage());
  m_wheelModel.Update(20_ms);

  m_driveSim.SetRawRotorPosition(m_wheelModel.GetAngularPosition() * 
                                kDriveEncoderReduction);
  m_driveSim.SetRotorVelocity(m_wheelModel.GetAngularVelocity() *
                              kDriveEncoderReduction);
  m_driveSim.SetRotorAcceleration(m_wheelModel.GetAngularAcceleration() *
                                  kDriveEncoderReduction);
}