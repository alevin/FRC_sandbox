// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package harkerrobolib.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Represents a command to be run indefinitely.
 *
 * @author Finn Frankis
 * @version Aug 17, 2018
 */
public class IndefiniteCommand extends CommandBase {

  /** {@inheritDoc} */
  @Override
  public boolean isFinished() {
    return false;
  }
}
