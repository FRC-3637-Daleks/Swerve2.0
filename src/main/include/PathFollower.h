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

#include <frc/controller/HolonomicDriveController.h>
#include <frc/trajectory/Trajectory.h>

#include "subsystems/Drivetrain.h"

class PathFollower
        : public frc2::CommandHelper<frc2::Command, PathFollower> {
    public:
    using pose_supplier_t =
        std::function<frc::Pose2d()>;
        /**

   * Creates a new ExampleCommand.
   *
   * @param trajectory The trajectory you want to follow
   * @param holonomicController The controller to be used to calculate 
   *                            the chassis speeds to follow the path
   * @param desiredPoseSupplier A function that returns the desired pose
   * @param output The command which uses the chassis speeds from
   *               the holonomic controller to convert them into
   *               raw module states, which it then sets the modules to
   * @param tolerance The tolerance for error
   * @param subsystem The subsystem used by this command.
   */
    explicit PathFollower(frc::Trajectory trajectory,
                frc::HolonomicDriveController holonomicController,
                frc::SwerveDriveKinematics<4>* driveKinematics,
                pose_supplier_t* desiredPoseSupplier,
                std::function<void(frc::ChassisSpeeds speeds)>* output,
                Drivetrain* subsystem, 
                frc::Pose2d tolerance = {0.0_m, 0.0_m, 0.0_rad});

    void Initialize() override;

    void Execute() override;

    bool isFinished();

    void End(bool interrupted) override;
    private:
    frc::Trajectory m_trajectory;
    frc::HolonomicDriveController m_holonomicController;
    frc::Pose2d m_tolerance;
    Drivetrain* m_driveSubsystem;
    frc::SwerveDriveKinematics<4>* m_kinematics;




    };


