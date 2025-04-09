package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

    public static final double kMaxSpeedMetersPerSecond = 4.5; // Maximum robot speed
    public static final double kMaxAcceleration = 3; // Maximum linear acceleration
    public static final double kMaxAngularSpeed = Math.PI * 1.2; // Maximum angular velocity (rad/s)
    public static final double kMaxAngularAcceleration = Math.PI * 2; // Maximum angular acceleration

    public static final double kMaxSpeedMetersPerSecondPathfind = 3; // Maximum robot speed
    public static final double kMaxAccelerationPathfind = 2; // Maximum linear acceleration
    public static final double kMaxAngularSpeedPathfind = Math.PI / 2; // Maximum angular velocity (rad/s)
    public static final double kMaxAngularAccelerationPathfind = 0.5; // Maximum angular acceleration

    public static final double kTrackWidth = 0.569; // Distance between left and right wheels (meters)
    public static final double kWheelBase = 0.517; // Distance between front and back wheels (meters)

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // CAN IDs for driving motors
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;

    // CAN IDs for turning motors
    public static final int kFrontRightTurningCanId = 10;
    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearRightTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 13;

    // CAN IDs for CANCoders (absolute encoders)
    public static final int kFrontRightcanCoderIDCanId = 19;
    public static final int kFrontLeftcanCoderIDCanId = 20;
    public static final int kRearRightcanCoderIDCanId = 17;
    public static final int kRearLeftcanCoderIDCanId = 21;

    // Offset values for CANCoders
    public static final float kFrontRightcanCoderOffset = 236.5f;
    public static final float kFrontLeftcanCoderOffset = 177.6f;
    public static final float kRearRightcanCoderOffset = 2.28f;
    public static final float kRearLeftcanCoderOffset = 90.3f;

    // Gyro configuration
    public static final boolean kGyroReversed = false;

    // Auto constants
    public static final double X_REEF_ALIGNMENT_P = 0.1;
    public static final double Y_REEF_ALIGNMENT_P = 0.1;
    public static final double ROT_REEF_ALIGNMENT_P = 0.005;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.435; // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16; // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 0.5;
    public static final double POSE_VALIDATION_TIME = 0.3;
  }

  /**
   * Constants related to individual swerve modules.
   */
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.101; // Wheel diameter in meters
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

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    // Motor idle modes
    public static final NeutralModeValue kDrivingMotorIdleMode = NeutralModeValue.Brake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    // Current limits for motors
    public static final int kDrivingMotorCurrentLimit = 70;
    public static final int kTurningMotorCurrentLimit = 70;
    public static final int kDrivingMotorStallCurrentLimit = 100;
    public static final int kTurningMotorStallCurrentLimit = 100;
  }

  public static final class ElevatorConstants {
    public static final int kElevatorMotorCanID = 5;
    public static final int kElevatorMotorCanID2 = 22;

    public static final boolean kElevatorMotorInverted = false;
    public static final boolean kElevatorMotorEncoderInverted = false;

    public static final double kElevatorMotorForwardSoftLimit = 4.26;
    public static final double kElevatorMotorReverseSoftLimit = 0;

    public static final double kElevatorGearRatio = 5; // Gear ratio

    // Conversion factor from motor rotations to elevator rise in cm
    public static final double kElevatorMotorSensorToMechRatio = kElevatorGearRatio;
    public static final double kElevatorMotorP = 2.25;
    public static final double kElevatorMotorI = 0;
    public static final double kElevatorMotorD = 0.1;
    public static final double kElevatorMaxSpeed = 0.85;
    public static final double kElevatorMaxSpeedDown = 0.25;
  }

  public static final class ArmConstants {
    public static final int kArmMotorCanID = 16;

    public static final boolean kArmMotorInverted = false;
    public static final boolean kArmMotorEncoderInverted = false;

    public static final double kArmMotorForwardSoftLimit = -300;
    public static final double kArmMotorReverseSoftLimit = -600;

    public static final double kArmMotorSensorToMechRatio = 500;

    public static final double kArmMotorP = 0.0025;
    public static final double kArmMotorI = 0;
    public static final double kArmMotorD = 0;
    public static final double kArmMotorMaxSpeed = 1;
  }

  public static final class ClimbConstants {
    public static final int kClimbMotorCanID = 21;

    public static final boolean kClimbMotorInverted = false;

    public static final double kClimbMotorForwardSoftLimit = 500;
    public static final double kClimbMotorReverseSoftLimit = 0;

    public static final double kClimbMotorSensorToMechRatio = 1 / 170;

    public static final double kClimbMotorP = 0.001;
    public static final double kClimbMotorI = 0;
    public static final double kClimbMotorD = 0;
    public static final double kClimbMotorMaxSpeed = 1;
  }

  public static final class ArmIntakeConstants {
    public static final int kArmIntakeMotorCanID = 7;
    public static final int kCoralArmIntakeSensorPort = 3;
    public static final int kAlgaeArmIntakeSensorPort = 2;

    public static final boolean kArmIntakeMotorInverted = true;
  }

  public static final class VisionConstants {
    public static AprilTagFieldLayout fieldLayout;
  }

  /**
   * Constants related to operator input.
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // Port for the driver's controller
    public static final double kDriveDeadband = 0.075; // Deadband for joystick input
  }
}