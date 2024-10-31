#include "PathFollower.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/math.h>

#include <iostream>
#include <numbers>
#include <random>

PathFollower::PathFollower(choreolib::ChoreoTrajectory trajectory,
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
  m_field->GetObject("Trajectory")->SetPoses(m_trajectory.GetPoses());
}

void PathFollower::Execute() {
    auto currentTime = m_timer.Get();
    auto desiredState = m_trajectory.Sample(currentTime);
    auto desiredPose = desiredState.GetPose();
    auto feedForward = desiredState.GetChassisSpeeds();
    m_driveSubsystem.DriveToPose(desiredPose, feedForward, {0.0_m, 0.0_m, 0_deg});
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  m_field->GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);
}

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.GetPoses()
    [m_trajectory.GetPoses().size() - 1];
  return m_driveSubsystem.AtPose(finalPose) && 
    m_driveSubsystem.IsStopped();
}

frc2::CommandPtr Drivetrain::FollowPathCommand(choreolib::ChoreoTrajectory trajectory) {
  return PathFollower{trajectory, *this}.ToPtr();
}