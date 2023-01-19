// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package harkerrobolib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/**
 * Represents a conditional command that uses a lambda to produce conditional values.
 *
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @since February 12, 2019
 */
public class ConditionalCommand extends edu.wpi.first.wpilibj2.command.ConditionalCommand {
  private BooleanSupplier condition;

  public ConditionalCommand(BooleanSupplier condition, Command trueCommand) {
    super(
        trueCommand,
        new PrintCommand("ConditionalCommand w/ no false command had false condition"),
        condition);
    this.condition = condition;
  }

  public ConditionalCommand(BooleanSupplier condition, Command trueCommand, Command falseCommand) {
    super(trueCommand, falseCommand, condition);
    this.condition = condition;
  }

  public boolean condition() {
    return condition.getAsBoolean();
  }
}
