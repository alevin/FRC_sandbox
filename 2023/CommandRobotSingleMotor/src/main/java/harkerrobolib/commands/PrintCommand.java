// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
