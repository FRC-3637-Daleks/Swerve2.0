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
                frc::HolonomicDriveController holonomicController,
                frc::SwerveDriveKinematics<4>* driveKinematics,
                pose_supplier_t* desiredPoseSupplier,
                std::function<void(frc::ChassisSpeeds speeds)>* output,
                Drivetrain* subsystem, 
                frc::Pose2d tolerance) 
        : m_trajectory{std::move(trajectory)},
          m_holonomicController{holonomicController}, 
          m_driveSubsystem{subsystem},  
          m_kinematics{driveKinematics},
          m_tolerance{tolerance}
{
    AddRequirements(subsystem);
};

void PathFollower::Initialize() {
    m_holonomicController.SetTolerance(m_tolerance);}

void PathFollower::Execute() {
    
}