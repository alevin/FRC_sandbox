// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package harkerrobolib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Cancels a command.
 *
 * @author Manan
 */
public class CancelCommand extends InstantCommand {

  private Command command;

  /**
   * Initializes a new CancelCommand to notify the Scheduler to stop a given command.
   *
   * @param c The command that will be canceled.
   */
  public CancelCommand(Command c) {
    command = c;
  }

  /** {@inheritDoc} */
  @Override
  public void initialize() {
    command.cancel();
  }
}
