// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
    //SpeedController m_frontRight = (SpeedController) new TalonSRX(12);
    m_myRobot = new DifferentialDrive(m_frontLeft,  m_frontRight);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
      DriverStation.reportWarning("NavX is connected", false);
  } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  

    CANStatus can_status = RobotController.getCANStatus();
    SmartDashboard.putNumber("Can bus utilization", can_status.percentBusUtilization);
    SmartDashboard.putNumber("Can bus recv errors", can_status.receiveErrorCount);
    SmartDashboard.putNumber("Can bus tx errors", can_status.transmitErrorCount);

    SmartDashboard.putNumber("RobotController Battery Voltage:", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("RobotController Input Voltage", RobotController.getInputVoltage());
    SmartDashboard.putNumber("RobotController Input Current", RobotController.getInputCurrent());

    SmartDashboard.putNumber("Left X", m_mController.getX(Hand.kLeft));
    SmartDashboard.putNumber("Left Y", m_mController.getY(Hand.kLeft));
    SmartDashboard.putNumber("Right X", m_mController.getX(Hand.kRight));
    SmartDashboard.putNumber("Right Y", m_mController.getY(Hand.kRight));


    //m1_Motor.set(ControlMode.PercentOutput, m_driverController.getY(Hand.kLeft) * 0.75 );

/* Display 6-axis Processed Angle Data                                      */
SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

         /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */
          
          SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
          
          SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
          SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
          SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
          SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
          SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
          SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
          SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
          SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
          
          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
          /* NOTE:  These values are not normally necessary, but are made available   */
          /* for advanced users.  Before using this data, please consider whether     */
          /* the processed data (see above) will suit your needs.                     */
          
          SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
          SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
          SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
          SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
          SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
          SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
          SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
          SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
          SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
          SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
          
          /* Omnimount Yaw Axis Information                                           */
          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
          SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
          SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
          
          /* Sensor Board Information                                                 */
          SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
          
          /* Quaternion Data                                                          */
          /* Quaternions are fascinating, and are the most compact representation of  */
          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
          /* from the Quaternions.  If interested in motion processing, knowledge of  */
          /* Quaternions is highly recommended.                                       */
          SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
          SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
          SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
          SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
          
          /* Connectivity Debugging Support                                           */
          SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
          SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
      

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   // m_myRobot.tankDrive(m_mController.getY(Hand.kLeft), m_mController.getY(Hand.kRight));
    m_myRobot.arcadeDrive(m_mController.getY(Hand.kLeft), m_mController.getX(Hand.kRight));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
