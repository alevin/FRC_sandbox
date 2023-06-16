package frc.robot;

public class Constants {
    public static final double GEAR_RATIO = 7.0;
    public static final double WHEEL_SIZE = 3.838;
    // public static final double WHEEL_SIZE = 4; //  wheel size from the doc for HiGrip wheels
  
    public static final double TICKS_PER_INCH = (2048 * GEAR_RATIO) / (WHEEL_SIZE * Math.PI);
  
  public static final int FR_DRIVE = 8;
  public static final int FR_STEER = 5;
  public static final int FR_OFFSET = 0;
  
  public static final int FL_DRIVE = 6;
  public static final int FL_STEER = 7;
  public static final int FL_OFFSET = 0;

  public static final int BL_DRIVE = 4;
  public static final int BL_STEER = 3;
  public static final int BL_OFFSET = 0;

  public static final int BR_DRIVE = 2;
  public static final int BR_STEER = 1;
  public static final int BR_OFFSET = 0;
}
