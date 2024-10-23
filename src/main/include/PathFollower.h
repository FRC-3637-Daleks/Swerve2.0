#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/DataLogManager.h>
#include <frc/I2C.h>
#include <frc/SPI.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/Commands.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/Trajectory.h>

#include "subsystems/Drivetrain.h"

class PathFollower
        : public frc2::CommandHelper<frc2::Command, PathFollower> {
    public:
    using pose_supplier_t =
        std::function<frc::Pose2d()>;

    using swerve_state_modifier_t = 
        std::function<void(std::array<frc::SwerveModuleState, 4>)>;
        /**

   * Creates a new ExampleCommand.
   *
   * @param trajectory The trajectory you want to follow
   * @param desiredPoseSupplier A function that returns the desired pose
   * @param subsystem The subsystem used by this command.
   */
    explicit PathFollower(frc::Trajectory trajectory,
                pose_supplier_t desiredPoseSupplier,
                Drivetrain &subsystem);

    void Execute() override;
    private:
    frc::Trajectory m_trajectory;
    frc::HolonomicDriveController m_holonomicController;
    frc::SwerveDriveKinematics<4> m_kinematics;
    pose_supplier_t m_desiredPoseSupplier;
    swerve_state_modifier_t m_output;
    Drivetrain& m_driveSubsystem;
    };


