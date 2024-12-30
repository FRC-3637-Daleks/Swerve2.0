#include "subsystems/ROSBridge.h"

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <frc/DriverStation.h>
#include <frc/RobotController.h>

ROSBridge::ROSBridge() {
  m_ntInst = nt::NetworkTableInstance::NetworkTableInstance::GetDefault();

  m_ntInst.StartClient4("RosDrivetrain");

  m_pubOdomTimestamp =
    m_ntInst.GetIntegerTopic("/Drivetrain/nt2ros/odom/timestamp").Publish();
  m_pubOdomPosLinear =
    m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/position/linear")
            .Publish();
  m_pubOdomPosAngular =
    m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/position/angular")
            .Publish();
  m_pubOdomVelLinear =
    m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/velocity/linear")
            .Publish();
  m_pubOdomVelAngular =
    m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/velocity/angular")
            .Publish();
  m_pubOdomAccLinear =
    m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/acceleration/linear")
            .Publish();

  m_subMapToOdomLinear =
    m_ntInst.GetDoubleArrayTopic("/Drivetrain/ros2nt/map2odom/linear")
            .Subscribe(wpi::array{0., 0., 0.});
  m_subMapToOdomAngular =
    m_ntInst.GetDoubleArrayTopic("/Drivetrain/ros2nt/map2odom/angular")
            .Subscribe(wpi::array{0., 0., 0.});

  m_fmsTable = m_ntInst.GetTable("FMSInfo");
}

void ROSBridge::CheckFMS() {
  using DS = frc::DriverStation;
  m_fmsTable->PutString("EventName", DS::GetEventName());
  m_fmsTable->PutString("GameSpecificMessage", DS::GetGameSpecificMessage());
  m_fmsTable->PutNumber("StationNumber", DS::GetLocation().value_or(0));
  m_fmsTable->PutNumber("MatchType", DS::GetMatchType());
  m_fmsTable->PutNumber("MatchNumber", DS::GetMatchNumber());
  m_fmsTable->PutNumber("ReplayNumber", DS::GetReplayNumber());
  m_fmsTable->PutBoolean("IsRedAlliance",
                         DS::GetAlliance() == DS::Alliance::kRed);
}

void ROSBridge::PubOdom(const frc::Pose2d &pose,
                        const frc::ChassisSpeeds &vel,
                        const units::second_t timestamp) {
  auto current_time_micros = units::microsecond_t{timestamp}.value();
  m_pubOdomTimestamp.Set(current_time_micros, current_time_micros);

  double pubPosLinear[3] = {units::meter_t{pose.X()}.value(),
                            units::meter_t{pose.Y()}.value(), 0};
  m_pubOdomPosLinear.Set(pubPosLinear, current_time_micros);

  double pubPosAngular[3] = {0, 0, pose.Rotation().Radians().value()};
  m_pubOdomPosAngular.Set(pubPosAngular, current_time_micros);

  double pubVelLinear[3] = {units::meters_per_second_t{vel.vx}.value(),
                            units::meters_per_second_t{vel.vy}.value(), 0};
  m_pubOdomVelLinear.Set(pubVelLinear, current_time_micros);

  double pubVelAngular[3] = {0, 0,
                             units::radians_per_second_t{vel.omega}.value()};
  m_pubOdomVelAngular.Set(pubVelAngular, current_time_micros);

  m_ntInst.Flush();
}

frc::Transform2d ROSBridge::GetMapToOdom() {
  // disgusting allocations, can be optimized
  auto orientation = m_subMapToOdomAngular.Get();
  auto offset = m_subMapToOdomLinear.Get();
  if (orientation.size() < 3) orientation = {0., 0., 0.};
  if (offset.size() < 3) offset = {0., 0., 0.};

  const units::meter_t x{offset[0]};
  const units::meter_t y{offset[1]};
  const units::degree_t theta{orientation[2]};

  return frc::Transform2d{x, y, theta};
}