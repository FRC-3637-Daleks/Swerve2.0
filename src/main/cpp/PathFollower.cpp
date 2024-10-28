#include "PathFollower.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>

#include <iostream>
#include <numbers>
#include <random>

PathFollower::PathFollower(frc::Trajectory trajectory,
                Drivetrain &subsystem) 
        : m_trajectory{std::move(trajectory)}, 
          m_driveSubsystem{subsystem},
          m_field{&m_driveSubsystem.GetField()}
{
    AddRequirements(&m_driveSubsystem);
};

void PathFollower::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

void PathFollower::Execute() {
    auto currentTime = m_timer.Get();
    auto desiredState = m_trajectory.Sample(currentTime);
    m_driveSubsystem.DriveToPose(desiredState, {0.0_m, 0.0_m, 0_deg});
    m_field->GetObject("Trajectory")->SetTrajectory(m_trajectory);
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  m_field->GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);
}

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.States() 
    [m_trajectory.States().size() - 1].pose;
  return m_driveSubsystem.AtPose(finalPose) && 
    m_driveSubsystem.IsStopped();
}

frc2::CommandPtr Drivetrain::FollowPathCommand(frc::Trajectory trajectory) {
  return PathFollower{trajectory, *this}.ToPtr();
}