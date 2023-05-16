// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.SwerveDriveModule;

public class SwerveModuleCommand extends CommandBase {
  // private SwerveDriveModule mDriveModule;

  public SwerveModuleCommand(final SwerveDriveModule driveModule) {
    // this.mDriveModule = driveModule;

    addRequirements(driveModule);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
