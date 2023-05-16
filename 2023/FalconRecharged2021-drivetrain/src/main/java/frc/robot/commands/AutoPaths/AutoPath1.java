// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryHelper;
import frc.robot.commands.swervedrive.Autonomous;
import frc.robot.commands.swervedrive.TurnToAngleProfiled;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPath1 extends SequentialCommandGroup {
  /**
   * Creates a new AutoPath1.
   *
   * <p>This is just an Auto used for general testing.
   */
  public AutoPath1(SwerveDriveSubsystem swerveDriveSubsystem) { // test forward path

    super(
        // new Autonomous(swerveDriveSubsystem,
        // TrajectoryHelper.createTestMultiPath().getTrajectory(),
        // TrajectoryHelper.createTestMultiPath().getAngle())

        new TurnToAngleProfiled(45, swerveDriveSubsystem).withTimeout(3),
        new Autonomous(
                swerveDriveSubsystem,
                TrajectoryHelper.createForwardPath().getTrajectory(),
                TrajectoryHelper.createForwardPath().getAngle())
            .withTimeout(1));
  }
}
