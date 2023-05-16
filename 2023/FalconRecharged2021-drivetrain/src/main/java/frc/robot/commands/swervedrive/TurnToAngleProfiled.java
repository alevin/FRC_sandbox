// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.*;

// JMH

/** A command that will turn the robot to the specified angle. */
public class TurnToAngleProfiled extends ProfiledPIDCommand {
  SwerveDriveSubsystem swerveDriveSubsystem;
  TrapezoidProfile.Constraints rampUpDown = new TrapezoidProfile.Constraints(10, 5);

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngleProfiled(double targetAngleDegrees, SwerveDriveSubsystem drive) {
    super(
        new ProfiledPIDController(
            Constants.ANGLE_PID_P,
            Constants.ANGLE_PID_I,
            Constants.ANGLE_PID_D, // need to tune this better
            new TrapezoidProfile.Constraints(
                Constants.MAX_ANGLE_VELOCITY, Constants.MAX_ANGLE_ACCELERATION)),

        // Close loop on heading
        drive::getGyroAngle2,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn branchrobot
        (output, setpoint) -> drive.holonomicDrive(0, 0, output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    swerveDriveSubsystem = drive;
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.TURN_TOLERANCE, 10);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    System.out.println("angle: " + swerveDriveSubsystem.getGyroAngle2());
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
