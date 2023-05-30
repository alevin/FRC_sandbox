// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoPaths.AutoPath1;
import frc.robot.commands.swervedrive.HolonomicDriveCommand;
import frc.robot.commands.swervedrive.ZeroNavX;
import frc.robot.subsystems.Drive.SwerveDriveModule;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class DaphneOneContainer {
  private final XboxController mXboxController;
  // private final XboxController mXboxController2;

  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public DaphneOneContainer() {
    // create all the subsystems needed in this robot
    SwerveDriveModule m0 =
        new SwerveDriveModule(
            0,
            new TalonSRX(DaphneOneConstants.FR_STEER),
            new TalonFX(DaphneOneConstants.FR_DRIVE),
            89);
    SwerveDriveModule m1 =
        new SwerveDriveModule(
            1,
            new TalonSRX(DaphneOneConstants.FL_STEER),
            new TalonFX(DaphneOneConstants.FL_DRIVE),
            -(95 + 12));
    SwerveDriveModule m2 =
        new SwerveDriveModule(
            2,
            new TalonSRX(DaphneOneConstants.BL_STEER),
            new TalonFX(DaphneOneConstants.BL_DRIVE),
            180 + 15 + 24);
    SwerveDriveModule m3 =
        new SwerveDriveModule(
            3,
            new TalonSRX(DaphneOneConstants.BR_STEER),
            new TalonFX(DaphneOneConstants.BR_DRIVE),
            -(90));

    swerveDriveSubsystem = new SwerveDriveSubsystem(m0, m1, m2, m3);

    // create the input controllers
    mXboxController = new XboxController(0);

    // setup any default commands
    swerveDriveSubsystem.setDefaultCommand(
        new HolonomicDriveCommand(swerveDriveSubsystem, mXboxController));
    // colorPanelSpinner.setDefaultCommand(new SpinnerCommand(colorPanelSpinner,
    // mXboxController2));
    // shooter.setDefaultCommand(new SpinShooterMotor());

    // configure the buttons
    configureButtonBindings();
    swerveDriveSubsystem.zeroGyro();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton buttonA = new JoystickButton(mXboxController,
    // XboxController.Button.kA.value);
    // JoystickButton buttonX = new JoystickButton(mXboxController,
    // XboxController.Button.kX.value);
    // JoystickButton buttonB = new JoystickButton(mXboxController,
    // XboxController.Button.kB.value);
    // JoystickButton buttonY = new JoystickButton(mXboxController,
    // XboxController.Button.kY.value);
    // JoystickButton buttonLB = new JoystickButton(mXboxController,
    // XboxController.Button.kLeftBumper.value);

    // JoystickButton buttonRB = new JoystickButton(mXboxController,
    // XboxController.Button.kBumperRight.value);

    JoystickButton buttonBack =
        new JoystickButton(mXboxController, XboxController.Button.kBack.value);
    // JoystickButton buttonStart = new JoystickButton(mXboxController,
    // XboxController.Button.kStart.value);

    buttonBack.onTrue(new ZeroNavX(swerveDriveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutoPath1(swerveDriveSubsystem);
  }
}
