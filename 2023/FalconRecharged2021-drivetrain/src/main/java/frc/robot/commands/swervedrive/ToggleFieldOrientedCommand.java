// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;

public class ToggleFieldOrientedCommand extends InstantCommand {
  SwerveDriveSubsystem swerveDriveSubsystem;
  /** Creates a new ToggleFieldOriented. */
  public ToggleFieldOrientedCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDriveSubsystem.setFieldOriented(!swerveDriveSubsystem.isFieldOriented());
  }
}
