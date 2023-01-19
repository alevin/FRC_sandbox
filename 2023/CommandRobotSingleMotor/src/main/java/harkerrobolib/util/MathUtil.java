// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package harkerrobolib.util;

/**
 * Contains a series of useful mathematical functions.
 *
 * @author Finn Frankis
 * @author Angela Jia
 * @version 10/16/18
 */
public final class MathUtil {
  /**
   * Linearly maps a value currently within a given range to another range.
   *
   * @param value the value to be mapped.
   * @param currentMin the current minimum possible value that value can take on.
   * @param currentMax the current maximum possible value that value can take on.
   * @param desiredMin the desired minimum possible value that value can take on.
   * @param desiredMax the desired maximum possible value that value can take on.
   * @return
   */
  public static double map(
      double value, double currentMin, double currentMax, double desiredMin, double desiredMax) {
    return (value - currentMin) * (desiredMax - desiredMin) / (currentMax - currentMin)
        + desiredMin;
  }

  /**
   * Maps a joystick input value between [-1, 1] to one where any input value between [-deadband,
   * deadband] is zero and anything outside of that range is mapped linearly from [0,1].
   *
   * @param inputValue the measured input value.
   * @param deadband the joystick's deadband.
   * @return the mapped joystick input.
   */
  public static double mapJoystickOutput(double inputValue, double deadband) {
    if (Math.abs(inputValue) <= deadband) {
      return 0;
    }
    return inputValue > 0
        ? map(inputValue, deadband, 1, 0, 1)
        : map(inputValue, -1, -deadband, -1, 0);
  }
}
