#include "swerve/PathFollower.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include <iostream>
#include <numbers>
#include <random>

PathFollower::PathFollower(trajectory_t trajectory, SwerveChassis& swerve)
  : m_trajectory{std::move(trajectory)}
  , m_swerve{swerve} {
  AddRequirements(&m_swerve);
}

void PathFollower::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_swerve.GetField().GetObject("Trajectory")->SetPoses(m_trajectory.GetPoses());
}

void PathFollower::Execute() {
  auto currentTime = m_timer.Get();
  if (auto desiredState =
    m_trajectory.SampleAt(currentTime, /* mirror */ false)) {
    auto desiredPose = desiredState->GetPose();
    auto feedForward = desiredState->GetChassisSpeeds();
    m_swerve.DriveToPose(desiredPose, feedForward,
      {0.0_m, 0.0_m, 0_deg});
  }
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  // teleport the trajectory "off-screen" once the command is complete
  m_swerve.GetField().GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);
}

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.GetFinalPose();
  return finalPose.has_value()
    && m_swerve.AtPose(finalPose.value())
    && m_swerve.IsStopped();
}