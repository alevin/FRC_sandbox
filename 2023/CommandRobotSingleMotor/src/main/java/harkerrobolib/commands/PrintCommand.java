// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package harkerrobolib.commands;

/**
 * Prints a given value to the console.
 *
 * @author Finn Frankis
 * @version 10/17/18
 */
public class PrintCommand extends CallMethodCommand {
  /**
   * Constructs a new PrintCommand.
   *
   * @param value the value to be printed
   */
  public PrintCommand(String value) {
    super(
        () -> {
          System.out.println(value);
        });
  }
}
