// Copyright (c) FIRST and other WPILib contributors
//2023 FRC 5507
// http://github.com/gwhs
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib5507.util;

/**
 * Holds universal constants for use throughout the library.
 *
 * @author Finn Frankis
 * @version 10/21/18
 */
public final class Constants {
  public static final int DEFAULT_TIMEOUT = 10;
  public static final int PID_PRIMARY = 0;
  public static final int PID_AUXILIARY = 1;

  public static final double FEET_PER_METER = 3.28084;
  public static final double METERS_PER_FOOT = 0.3048;

  public static final double JOYSTICK_DEADBAND = 0.1;
  public static final double TRIGGER_DEADBAND = 0.1;
}
