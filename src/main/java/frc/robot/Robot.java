// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // private final TalonSRX m1_Motor = new TalonSRX(12);
  // private final XboxController m_driverController = new XboxController(0);

  private DifferentialDrive m_myRobot;
  private XboxController m_mController;

  AHRS ahrs;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_mController = new XboxController(0);
    CANSparkMax m_frontLeft = new CANSparkMax(8, MotorType.kBrushless);
    CANSparkMax m_frontRight = new CANSparkMax(25, MotorType.kBrushed);
    // SpeedController m_frontRight = (SpeedController) new TalonSRX(12);
    m_myRobot = new DifferentialDrive(m_frontLeft, m_frontRight);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      ahrs = new AHRS(SPI.Port.kMXP);
      DriverStation.reportWarning("NavX is connected", false);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    CANStatus can_status = RobotController.getCANStatus();
    SmartDashboard.putNumber("Can_Bus/Can bus utilization", can_status.percentBusUtilization);
    SmartDashboard.putNumber("Can_Bus/Can bus recv errors", can_status.receiveErrorCount);
    SmartDashboard.putNumber("Can_Bus/Can bus tx errors", can_status.transmitErrorCount);

    SmartDashboard.putNumber("RobotController/Battery Voltage:", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("RobotController/Input Voltage", RobotController.getInputVoltage());
    SmartDashboard.putNumber("RobotController/Input Current", RobotController.getInputCurrent());

    SmartDashboard.putNumber("Xbox0/Left X", m_mController.getX(Hand.kLeft));
    SmartDashboard.putNumber("Xbox0/Left Y", m_mController.getY(Hand.kLeft));
    SmartDashboard.putNumber("Xbox0/Right X", m_mController.getX(Hand.kRight));
    SmartDashboard.putNumber("Xbox0/Right Y", m_mController.getY(Hand.kRight));

    // m1_Motor.set(ControlMode.PercentOutput, m_driverController.getY(Hand.kLeft) *
    // 0.75 );

    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("NavX/IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("NavX/IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("NavX/IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("NavX/IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("NavX/IMU_Roll", ahrs.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("NavX/IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("NavX/IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

    SmartDashboard.putNumber("NavX/IMU_TotalYaw", ahrs.getAngle());
    SmartDashboard.putNumber("NavX/IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("NavX/IMU_Accel_X", ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber("NavX/IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean("NavX/IMU_IsMoving", ahrs.isMoving());
    SmartDashboard.putBoolean("NavX/IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("NavX/Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("NavX/Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("NavX/Displacement_X", ahrs.getDisplacementX());
    SmartDashboard.putNumber("NavX/Displacement_Y", ahrs.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("NavX/RawGyro_X", ahrs.getRawGyroX());
    SmartDashboard.putNumber("NavX/RawGyro_Y", ahrs.getRawGyroY());
    SmartDashboard.putNumber("NavX/RawGyro_Z", ahrs.getRawGyroZ());
    SmartDashboard.putNumber("NavX/RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("NavX/RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("NavX/RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("NavX/RawMag_X", ahrs.getRawMagX());
    SmartDashboard.putNumber("NavX/RawMag_Y", ahrs.getRawMagY());
    SmartDashboard.putNumber("NavX/RawMag_Z", ahrs.getRawMagZ());
    SmartDashboard.putNumber("NavX/IMU_Temp_C", ahrs.getTempC());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString("NavX/YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("NavX/YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("NavX/FirmwareVersion", ahrs.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("NavX/QuaternionW", ahrs.getQuaternionW());
    SmartDashboard.putNumber("NavX/QuaternionX", ahrs.getQuaternionX());
    SmartDashboard.putNumber("NavX/QuaternionY", ahrs.getQuaternionY());
    SmartDashboard.putNumber("NavX/QuaternionZ", ahrs.getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("NavX/IMU_Byte_Count", ahrs.getByteCount());
    SmartDashboard.putNumber("NavX/IMU_Update_Count", ahrs.getUpdateCount());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // m_myRobot.tankDrive(m_mController.getY(Hand.kLeft),
    // m_mController.getY(Hand.kRight));
    m_myRobot.arcadeDrive(m_mController.getY(Hand.kLeft), m_mController.getX(Hand.kRight));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
