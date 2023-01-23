package frc.lib5507.hardwareWrappers.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib5507.hardwareWrappers.Gyro.NavX.RealNavx;
import frc.lib5507.hardwareWrappers.Gyro.Pigeon2.RealPigeon;

public class WrapperedGyro {

  AbstractGyro gyro;
  double offset_rad = 0;

  // public enum GyroType {
  //   NAVX,
  //   PIGEON
  // }

  private double curAngle_rad;

  // public WrapperedGyro(GyroType type) {
  public WrapperedGyro(String type) {
    if (type == "NAVX") {
      gyro = new RealNavx();
    } else if (type == "PIGEON") {
      gyro = new RealPigeon();
    } else {
      gyro = null;
    }
  }

  public void update() {
    // Gyros are inverted in reference frame (positive clockwise)
    // and we maintain our own offset in code when rezeroing.
    curAngle_rad = gyro.getRawAngle() * -1.0 + offset_rad;
  }

  public void reset(double curAngle_rad) {
    offset_rad = curAngle_rad;
    gyro.reset();
  }

  public void calibrate() {
    gyro.calibrate();
  }

  public double getRate_radpersec() {
    return gyro.getRate();
  }

  public double getAngle_rad() {
    return curAngle_rad;
  }

  public boolean isConnected() {
    return gyro.isConnected();
  }

  public void setYaw(double angleDeg) {
    // new Rotation2d(this.getAngle_rad());

  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(this.getAngle_rad());
  }
}
