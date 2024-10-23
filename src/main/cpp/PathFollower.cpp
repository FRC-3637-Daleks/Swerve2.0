#include "PathFollower.h"

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

PathFollower::PathFollower(frc::Trajectory trajectory,
                pose_supplier_t desiredPoseSupplier,
                Drivetrain &subsystem) 
        : m_trajectory{std::move(trajectory)},
          m_holonomicController{m_driveSubsystem.holoThingy()}, 
          m_driveSubsystem{subsystem},
          m_desiredPoseSupplier{desiredPoseSupplier},
          m_output{
            [this] (auto states) {m_driveSubsystem.SetModuleStates(states);}},  
          m_kinematics{m_driveSubsystem.kineThingy()}
{
    AddRequirements(&m_driveSubsystem);
};

void PathFollower::Execute() {
  frc2::cmd::Run([this] {
    frc2::SwerveControllerCommand<4>{
      m_trajectory, m_desiredPoseSupplier,
      m_kinematics, m_holonomicController,
      m_output, {&m_driveSubsystem}};})
      .Until([this] {return m_driveSubsystem.AtPose(
      m_desiredPoseSupplier());});
}