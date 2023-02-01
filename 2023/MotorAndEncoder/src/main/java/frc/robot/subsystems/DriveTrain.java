// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  //  DifferentialDrive drive;
  WPI_TalonFX m1_Motor;
  private final BoreEncoder boreEncoder;

  // WPI_TalonSRX m2_Motor;
  /** Creates a new DriveTrain. */
  public DriveTrain() {

    m1_Motor = new WPI_TalonFX(Constants.MOTOR1);
    boreEncoder = new BoreEncoder();
    // m2_Motor = new WPI_TalonSRX (Constants.MOTOR2);
    // m1_Motor.configPeakCurrentLimit(5);
    // m1_Motor.configPeakCurrentDuration(100);
    // m1_Motor.configContinuousCurrentLimit(3);

    // drive = new DifferentialDrive(m1_Motor, m2_Motor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWithJoystick(XboxController controller) {
    m1_Motor.set(ControlMode.PercentOutput, controller.getLeftY() * Constants.SPEED);
    // System.out.println("getY = " + controller.getLeftY());
    boreEncoder.periodic();
  }

  public void driveForward() {
    m1_Motor.set(ControlMode.PercentOutput, 0.5 * Constants.SPEED);
  }

  public void stop() {
    m1_Motor.set(ControlMode.PercentOutput, 0);
  }
}
