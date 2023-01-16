// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package harkerrobolib.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Executes a given runnable once.
 *
 * @author Finn Frankis
 * @version 10/18/18
 */
public class CallMethodCommand extends InstantCommand {
  Runnable method;

  public CallMethodCommand(Runnable method) {
    this.method = method;
  }

  public void initialize() {
    method.run();
  }
}
