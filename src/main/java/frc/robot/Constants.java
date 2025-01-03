package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.
 * All constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 */
public final class Constants {

  /**
   * Constants related to the drive subsystem.
   */
  public static final class DriveConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.8; // Maximum robot speed
    public static final double kMaxAcceleration = 4; // Maximum linear acceleration
    public static final double kMaxAngularSpeed = Math.PI; // Maximum angular velocity (rad/s)
    public static final double kMaxAngularAcceleration = 3; // Maximum angular acceleration

    public static final double kTrackWidth = 0.679; // Distance between left and right wheels (meters)
    public static final double kWheelBase = 0.519; // Distance between front and back wheels (meters)

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // CAN IDs for driving motors
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    // CAN IDs for turning motors
    public static final int kFrontLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 16;
    public static final int kRearLeftTurningCanId = 10;
    public static final int kRearRightTurningCanId = 15;

    // CAN IDs for CANCoders (absolute encoders)
    public static final int kFrontLeftcanCoderIDCanId = 20;
    public static final int kFrontRightcanCoderIDCanId = 19;
    public static final int kRearLeftcanCoderIDCanId = 21;
    public static final int kRearRightcanCoderIDCanId = 17;

    // Offset values for CANCoders
    public static final float kFrontLeftcanCoderOffset = 46.75f;
    public static final float kFrontRightcanCoderOffset = 3.2f;
    public static final float kRearLeftcanCoderOffset = -22.93f;
    public static final float kRearRightcanCoderOffset = 178.5f;

    // Gyro configuration
    public static final boolean kGyroReversed = false;
    public static final double kGyroOffset = 0.0;
  }

  /**
   * Constants related to individual swerve modules.
   */
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.103; // Wheel diameter in meters
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // Circumference

    public static final double kDrivingMotorReduction = 6.5; // Gear reduction for driving motor
    public static final double kTurningMotorReduction = 10.04; // Gear reduction for turning motor

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction);
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0; // Per second

    public static final double kTurningEncoderPositionPIDMinInput = 0;
    public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI; // Full rotation in radians

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kTurningMotorReduction;
    public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60; // Per second

    // PID coefficients for driving and turning motors
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

    // Motor idle modes
    public static final NeutralModeValue kDrivingMotorIdleMode = NeutralModeValue.Brake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    // Current limits for motors
    public static final int kDrivingMotorCurrentLimit = 60;
    public static final int kTurningMotorCurrentLimit = 60;
    public static final int kDrivingMotorStallCurrentLimit = 100;
    public static final int kTurningMotorStallCurrentLimit = 100;
  }

  /**
   * Constants related to operator input.
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // Port for the driver's controller
    public static final double kDriveDeadband = 0.1; // Deadband for joystick input
  }
}