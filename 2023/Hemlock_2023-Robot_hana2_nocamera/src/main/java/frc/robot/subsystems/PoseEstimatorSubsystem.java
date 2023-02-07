package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;

//TODO: import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

//TODO: import frc.robot.photonvision.EstimatedRobotPose;
//TODO: import frc.robot.photonvision.PhotonPoseEstimator;
//TODO: import frc.robot.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.util.FieldConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

  //TODO: private final PhotonCamera photonCamera;
  private final DrivetrainSubsystem drivetrainSubsystem;
  //TODO: private final PhotonPoseEstimator photonPoseEstimator;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(5));
  // private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.025, 0.025,
  // Units.degreesToRadians(1));

  //TODO: private Optional<EstimatedRobotPose> photonEstimatedRobotPose = Optional.empty();

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  // private static final Vector<N3> visionMeasurementStdDevs =
  // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  // private static final Vector<N3> visionMeasurementStdDevs =
  // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final ArrayList<Double> xValues = new ArrayList<Double>();
  private final ArrayList<Double> yValues = new ArrayList<Double>();

  private final Field2d field2d = new Field2d();

  public PoseEstimatorSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
    //TODO: public PoseEstimatorSubsystem(PhotonCamera photonCamera, DrivetrainSubsystem drivetrainSubsystem) {
    // TODO: this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      // var alliance = Alliance.Blue;
      layout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
          : OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    // TODO: photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, this.photonCamera,ROBOT_TO_CAMERA);

    poseEstimator = new SwerveDrivePoseEstimator(
        DrivetrainConstants.KINEMATICS,
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    //TODO: photonEstimatedRobotPose = photonPoseEstimator.update();
    // if (photonEstimatedRobotPose.isPresent()) {
    //   EstimatedRobotPose pose = photonEstimatedRobotPose.get();
    //   // Max distance you want a tag to be read at. Found issues after 15 feet away
    //   // from tag while moving.
    //   // if (Math.hypot(pose.estimatedPose.getX(), pose.estimatedPose.getY()) < 5.25)
    //   // {
    //   // Error with WPI code https://github.com/wpilibsuite/allwpilib/issues/4952
    //   try {
    //     try {
    //       poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    //     } catch (NoSuchElementException e) {
    //     }
    //   } catch (ConcurrentModificationException e) {
    //   }
    // }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions());

    // Conversion so robot appears where it actually is on field instead of always
    // on blue.
    // xValues.add(getCurrentPose().getX());
    // yValues.add(getCurrentPose().getY());
    // double xAverage = xValues.stream().mapToDouble(a ->
    // a).average().getAsDouble();
    // double yAverage = yValues.stream().mapToDouble(a ->
    // a).average().getAsDouble();
    // double summation = 0.0;
    // for (int i = 0; i < xValues.size(); i++) {
    // summation += (Math.pow(xValues.get(i) - xAverage, 2) +
    // Math.pow(yValues.get(i) - yAverage, 2));
    // }
    // double RMS = Math.sqrt((1.0 / (double) xValues.size() * summation));
    // System.out.println("RMS: " + RMS);
    if (DriverStation.getAlliance() == Alliance.Red) {
      field2d.setRobotPose(new Pose2d(FieldConstants.fieldLength - getCurrentPose().getX(),
          FieldConstants.fieldWidth - getCurrentPose().getY(),
          new Rotation2d(getCurrentPose().getRotation().getRadians() + Math.PI)));
    } else {
      field2d.setRobotPose(getCurrentPose());
    }
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public void addTrajectory(PathPlannerTrajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  /**
   * Resets the holonomic rotation of the robot (gyro last year)
   * This would be used if Apriltags are not getting accurate pose estimation
   */
  public void resetHolonomicRotation() {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(0),
        drivetrainSubsystem.getModulePositions(),
        getCurrentPose());
  }

  public void resetPoseRating() {
    xValues.clear();
    yValues.clear();
  }
}