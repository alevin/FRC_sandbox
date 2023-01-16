// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
