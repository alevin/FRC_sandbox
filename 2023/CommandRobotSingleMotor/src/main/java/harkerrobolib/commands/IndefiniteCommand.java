// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
