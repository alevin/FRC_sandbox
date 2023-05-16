// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.HolonomicDrivetrain;

public class AdjustFieldOrientedAngleCommand extends CommandBase {
  public static final double ADJUSTMENT_AMOUNT = 5;

  private final HolonomicDrivetrain mDrivetrain;
  private final boolean mIncreaseAngle;

  public AdjustFieldOrientedAngleCommand(HolonomicDrivetrain drivetrain, boolean increaseAngle) {
    mDrivetrain = drivetrain;
    mIncreaseAngle = increaseAngle;
  }

  @Override
  public void execute() {
    if (mIncreaseAngle) {
      mDrivetrain.setAdjustmentAngle(mDrivetrain.getAdjustmentAngle() + ADJUSTMENT_AMOUNT);
    } else {
      mDrivetrain.setAdjustmentAngle(mDrivetrain.getAdjustmentAngle() - ADJUSTMENT_AMOUNT);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
