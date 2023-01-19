// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import harkerrobolib.wrappers.HSFalcon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonFX m1_Motor;

  /** Creates a new shooter. */
  public Shooter() {

    m1_Motor = new WPI_TalonFX(Constants.MOTOR1, "CAN_Network");

    /**
     * Configure the current limits that will be used Stator Current is the current that passes
     * through the motor stators. Use stator current limits to limit rotor acceleration/heat
     * production Supply Current is the current that passes into the controller from the supply Use
     * supply current limits to prevent breakers from tripping
     *
     * <p>enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
     */
    m1_Motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 3, 1.0));
    m1_Motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 1, 2, 0.5));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(XboxController controller) {
    // m1_Motor.set(ControlMode.PercentOutput, controller.getLeftY() * Constants.SPEED);
    m1_Motor.set(ControlMode.PercentOutput, controller.getRightTriggerAxis() * Constants.SPEED);
    // System.out.println("getY = " + controller.getLeftY());
  }

  public void driveForward() {
    m1_Motor.set(ControlMode.PercentOutput, 0.5 * Constants.SPEED);
  }

  public void stop() {
    m1_Motor.set(ControlMode.PercentOutput, 0);
  }
}
