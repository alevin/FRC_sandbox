package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonSRXConfiguration swerveAngleSRXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public TalonSRXConfiguration swerveTalonSRXConfig;

    public CTREConfigs(){
        swerveAngleSRXConfig = new TalonSRXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveTalonSRXConfig = new TalonSRXConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleSRXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleSRXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleSRXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleSRXConfig.slot0.kF = Constants.Swerve.angleKF;
        //swerveAngleSRXConfig.SupplyCurrentLimit  = angleSupplyLimit;
        swerveAngleSRXConfig.peakCurrentLimit = 10;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        
        /* Swerve Talon SRX Configuration */

        // in SwerveModule.java
    }
}