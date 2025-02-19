#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <choreo/trajectory/Trajectory.h>

#include "subsystems/Drivetrain.h"

class PathFollower
        : public frc2::CommandHelper<frc2::Command, PathFollower> {
public:
    using pose_supplier_t =
        std::function<frc::Pose2d()>;

    using swerve_state_modifier_t = 
        std::function<void(std::array<frc::SwerveModuleState, 4>)>;

    using trajectory_t = choreo::Trajectory<choreo::SwerveSample>;

    /**
   * Constructor
   *
   * @param trajectory The trajectory you want to follow
   * @param desiredPoseSupplier A function that returns the desired pose
   * @param subsystem The subsystem used by this command.
   */
    PathFollower(trajectory_t trajectory, Drivetrain &subsystem);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    virtual bool IsFinished();

    static inline void registerCommand(std::string name,
            std::shared_ptr<frc2::Command> command) {
        GetNamedCommands().emplace(name, command);
    }
 
    static inline void registerCommand(std::string name,
            frc2::CommandPtr &&command) {
        registerCommand(name,
                std::shared_ptr < frc2::Command
                        > (std::move(command).Unwrap()));
    }
 
    static inline bool hasCommand(std::string name) {
        return GetNamedCommands().contains(name);
    }
 
    static frc2::Command* getCommand(std::string name);
 
    static std::unordered_map<std::string, std::shared_ptr<frc2::Command>>& GetNamedCommands()
        {return m_namedCommands;}

private:
    static std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_namedCommands;
    static std::unordered_map<std::string, frc::Pose2d> m_eventPoses;
    trajectory_t m_trajectory;
    Drivetrain& m_driveSubsystem;
    frc::Timer m_timer;
    frc::Field2d* m_field;
};


