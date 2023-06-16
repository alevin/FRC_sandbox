// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private DaphneOneContainer m_daphneOneContainer;
  private TestbedContainer m_testbedContainer;

  private SendableChooser<Command> autoChooser;

  public static final String DAPHNE1 = "daphne1";
  public static final String TESTBED = "testbed";

  private static final String ROBOT_TYPE =
      DAPHNE1; // change this line to either "DAPHNE1" or "DAPHNE2" to switch
  // between
  // configurations.

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our DaphneTwoContainer. This will perform all our button
    // bindings, and put our
    // autonomous chooser on the dashboard.
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
    Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    switch (ROBOT_TYPE) {
      case DAPHNE1:
        m_daphneOneContainer = new DaphneOneContainer();
        // m_daphneOneContainer.getLimelight().setCamMode(); // not sure what this did
        // in
        // master branch
        m_autonomousCommand = m_daphneOneContainer.getAutonomousCommand();
        break;
      case TESTBED:
        m_testbedContainer = new TestbedContainer();
        m_autonomousCommand = m_testbedContainer.getAutonomousCommand();
        break;
      default:
        // unexpected, will crash later
        break;
    }
    autoChooser = new SendableChooser<Command>();

    // autoChooser.addOption("Move Forward 1", new
    // Autonomous(m_daphneTwoContainer.createAutonomousPath()));
    // autoChooser.addOption("Move Forward 2", new
    // Autonomous(m_daphneTwoContainer.createAutonomousPath1()));
    // autoChooser.addOption("Move Forward 3", new
    // Autonomous(m_daphneTwoContainer.createAutonomousPath2()));

    SmartDashboard.putData(autoChooser);

    Logger.getInstance()
        .start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
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
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your {@link DaphneTwoContainer} class.
   */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
