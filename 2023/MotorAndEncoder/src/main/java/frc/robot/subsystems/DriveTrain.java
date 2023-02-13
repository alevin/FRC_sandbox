// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
// import edu.wpi.first.wpilibj.SpeedController;

public class DriveTrain extends SubsystemBase {
  // DifferentialDrive drive;

  final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */
  final TalonFXInvertType kInvertType =
      TalonFXInvertType.Clockwise; // <<< What direction you want "forward/up" to be.
  final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;

  WPI_TalonFX m1_Motor;
  private final BoreEncoder boreEncoder;

  // WPI_TalonSRX m2_Motor;
  /** Creates a new DriveTrain. */
  public DriveTrain() {

    m1_Motor = new WPI_TalonFX(Constants.MOTOR1);
    m1_Motor.configFactoryDefault();
    // m1_Motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    // m1_Motor.setSensorPhase(false);
    // m1_Motor.configNeutralDeadband(0.001, 30);
    // m1_Motor.setSelectedSensorPosition(0, 0, 30);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m1_Motor.configAllSettings(configs);
    m1_Motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    m1_Motor.setInverted(kInvertType);
    m1_Motor.setNeutralMode(kBrakeDurNeutral);

    boreEncoder = new BoreEncoder();

    // double talonVelocity = m1_Motor.getSelectedSensorVelocity();
    // double talonPosition = m1_Motor.getSelectedSensorPosition();
    // double talonOutput = m1_Motor.getMotorOutputPercent();
    /* get the selected sensor for PID0 */

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
    double lefty = controller.getLeftY();
    m1_Motor.set(ControlMode.PercentOutput, lefty * Constants.SPEED);
    Logger.getInstance().recordOutput("XBoxController/LeftY", lefty);

    double appliedMotorOutput = m1_Motor.getMotorOutputPercent();
    double selSenPos = m1_Motor.getSelectedSensorPosition(0); /* position units */
    double selSenVel = m1_Motor.getSelectedSensorVelocity(0); /* position units per 100ms */

    double pos_Rotations = (double) selSenPos / kUnitsPerRevolution;
    double vel_RotPerSec =
        (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
    // double vel_RotPerMin = vel_RotPerSec * 60.0;

    Logger.getInstance().recordOutput("Talon/SensorVelocity", selSenVel);
    Logger.getInstance().recordOutput("Talon/RotatopnPerSec", vel_RotPerSec);
    Logger.getInstance().recordOutput("Talon/SensorPosition", selSenPos);
    Logger.getInstance().recordOutput("Talon/Rotations", pos_Rotations);
    Logger.getInstance().recordOutput("Talon/appliedMotorOutput", appliedMotorOutput);

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
