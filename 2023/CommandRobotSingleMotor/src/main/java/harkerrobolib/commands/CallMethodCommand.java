// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
