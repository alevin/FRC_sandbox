package frc.lib5507.hardwareWrappers.Gyro.Pigeon2;

// import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.DrivetrainConstants.CANIVORE_NAME;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.lib5507.hardwareWrappers.Gyro.AbstractGyro;

public class RealPigeon extends AbstractGyro {

  // AHRS ahrs;
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID, CANIVORE_NAME);

  public RealPigeon() {
    this.calibrate();
  }

  @Override
  public void reset() {
    pigeon.reset();
  }

  @Override
  public void calibrate() {
    System.out.println("======================================================");
    System.out.println("== GYRO: CALIBRATION IN PROCESS, DO NOT MOVE ROBOT...");
    pigeon.calibrate();
    System.out.println("== ... Complete!");
    System.out.println("======================================================");
  }

  @Override
  public double getRate() {
    return Units.degreesToRadians(pigeon.getRate());
  }

  @Override
  public double getRawAngle() {
    return Units.degreesToRadians(pigeon.getAngle());
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void setYaw(double angleDeg) {
    pigeon.setYaw(angleDeg);
  }

  // @Override
  //   public Rotation2d getGyroscopeRotation() {
  //   return pigeon.getRotation2d();
  //   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
  //   // the angle increase.
  //   // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  // }
}
