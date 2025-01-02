package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final class DriveConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAcceleration = 4;
    public static final double kMaxAngularSpeed = 1 * Math.PI;
    public static final double kMaxAngularAcceleration = 3;

    public static final double kTrackWidth = 0.679;
    public static final double kWheelBase = 0.519;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 16;
    public static final int kRearLeftTurningCanId = 10;
    public static final int kRearRightTurningCanId = 15;

    public static final int kFrontLeftcanCoderIDCanId = 20;
    public static final int kFrontRightcanCoderIDCanId = 19;
    public static final int kRearLeftcanCoderIDCanId = 21;
    public static final int kRearRightcanCoderIDCanId = 17;

    public static final float kFrontLeftcanCoderOffset = 46.75f;
    public static final float kFrontRightcanCoderOffset = 3.2f;
    public static final float kRearLeftcanCoderOffset = -22.93f;
    public static final float kRearRightcanCoderOffset = 178.5f;

    public static final boolean kGyroReversed = false;
    public static final double kGyroOffset = 0.0;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.103;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDrivingMotorReduction = 6.5;
    public static final double kTurningMotorReduction = 10.04;

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction);
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0;

    public static final double kTurningEncoderPositionPIDMinInput = 0;
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI * 2;

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kTurningMotorReduction;
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / (60 * kTurningMotorReduction);

    public static final double kDrivingP = 0.2;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0; 
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1; 

    public static final double kTurningP = 0.75;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake; 
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 60;
    public static final int kTurningMotorCurrentLimit = 60;
    public static final int kDrivingMotorStallCurrentLimit = 100;
    public static final int kTurningMotorStallCurrentLimit = 100;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }
}