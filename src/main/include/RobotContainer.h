// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc/smartdashboard/SendableChooser.h>

#include <numbers>

#include "subsystems/Drivetrain.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetDisabledCommand();
  frc2::CommandPtr GetAutonomousCommand();

public:
  // Replace with CommandPS4Controller or CommandJoystick if needed

  frc2::CommandJoystick m_swerveController;

  // Button Triggers are defined here.

  frc2::Trigger DriveFwdTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 0; }};

  frc2::Trigger DriveStrafeLeftTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 270; }};

  frc2::Trigger DriveStrafeRightTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 90; }};

  frc2::Trigger DriveRevTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 180; }};

  // The robot's subsystems are defined here...

  Drivetrain m_swerve;

  bool m_isRed;

public:
  void ConfigureBindings();
  void ConfigureDashboard();
  void ConfigureAuto();
};