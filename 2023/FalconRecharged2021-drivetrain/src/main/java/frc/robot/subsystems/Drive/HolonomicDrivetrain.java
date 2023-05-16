// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

public abstract class HolonomicDrivetrain extends Drivetrain {

  private double mAdjustmentAngle = 0;
  private boolean mFieldOriented = true;

  public double getAdjustmentAngle() {
    return mAdjustmentAngle;
  }

  public abstract double getGyroAngle();

  public abstract void holonomicDrive(double forward, double strafe, double rotation);

  public abstract void swapPIDSlot(int slot);

  public boolean isFieldOriented() {
    return mFieldOriented;
  }

  public void setAdjustmentAngle(double adjustmentAngle) {
    mAdjustmentAngle = adjustmentAngle;
  }

  public void setFieldOriented(boolean fieldOriented) {
    mFieldOriented = fieldOriented;
  }

  public abstract void stopDriveMotors();

  public void zeroGyro() {
    setAdjustmentAngle(getGyroAngle() + getAdjustmentAngle());
  }
}
