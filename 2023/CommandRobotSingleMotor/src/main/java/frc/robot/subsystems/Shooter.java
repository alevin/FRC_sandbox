// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import harkerrobolib.wrappers.HSFalcon;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX m1_Motor;

  /** Creates a new shooter. */
  public Shooter() {

    m1_Motor = new TalonFX(Constants.MOTOR1, "CAN_Network");

    /**
     * Configure the current limits that will be used Stator Current is the current that passes
     * through the motor stators. Use stator current limits to limit rotor acceleration/heat
     * production Supply Current is the current that passes into the controller from the supply Use
     * supply current limits to prevent breakers from tripping
     *
     * <p>enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
     */
    // m1_Motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 3, 1.0));
    //  m1_Motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 1, 2, 0.5));

    CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    m_currentLimits.SupplyCurrentLimit = 1; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = 4; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    m_currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

    toConfigure.CurrentLimits = m_currentLimits;

    m1_Motor.getConfigurator().apply(toConfigure);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(XboxController controller) {
    // m1_Motor.set(ControlMode.PercentOutput, controller.getLeftY() * Constants.SPEED);
    DutyCycleOut m_output = new DutyCycleOut(0);
    m1_Motor.setControl(m_output.withOutput(controller.getRightTriggerAxis() * Constants.SPEED));
    // System.out.println("getY = " + controller.getLeftY());
  }

  public void driveForward() {
    DutyCycleOut m_output = new DutyCycleOut(0);
    /// m1_Motor.set(ControlMode.PercentOutput, 0.5 * Constants.SPEED);
    m1_Motor.setControl(m_output.withOutput(0.5 * Constants.SPEED));
  }

  public void stop() {
    DutyCycleOut m_output = new DutyCycleOut(0);
    m1_Motor.setControl(m_output.withOutput(0));
  }
}
