// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class runShooter extends CommandBase {
  private final Shooter shooter;

  /** Creates a new runShooter. */
  public runShooter(Shooter dt) {
    shooter = dt;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    // TODO: Called when the command is initially scheduled.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runShooter(RobotContainer.driverJoystick);
  }

  @Override
  public void end(boolean interrupted) {
    // TODO: Called once the command ends or is interrupted.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
