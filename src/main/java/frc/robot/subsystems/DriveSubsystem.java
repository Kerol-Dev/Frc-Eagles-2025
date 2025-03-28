package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.pathfind.FieldPositions;
import frc.robot.subsystems.pathfind.PathfindType;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The DriveSubsystem class manages the swerve drive system of the robot.
 * It controls the four swerve modules and handles field-relative and
 * robot-relative driving.
 */
@AutoLog
public class DriveSubsystem extends SubsystemBase {
  // ONLY FOR SIMULATION

  // public static final SimulatedSwerveModule m_frontLeft = new
  // SimulatedSwerveModule(
  // 0.1);

  // public static final SimulatedSwerveModule m_frontRight = new
  // SimulatedSwerveModule(
  // 0);

  // public static final SimulatedSwerveModule m_rearLeft = new
  // SimulatedSwerveModule(
  // 0);

  // public static final SimulatedSwerveModule m_rearRight = new
  // SimulatedSwerveModule(
  // 0);

  // Swerve modules for each corner of the robot
  public static final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftcanCoderIDCanId,
      false,
      true,
      DriveConstants.kFrontLeftcanCoderOffset,
      false);

  public static final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightcanCoderIDCanId,
      false,
      true,
      DriveConstants.kFrontRightcanCoderOffset,
      false);

  public static final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftcanCoderIDCanId,
      false,
      true,
      DriveConstants.kRearLeftcanCoderOffset,
      false);

  public static final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightcanCoderIDCanId,
      false,
      true,
      DriveConstants.kRearRightcanCoderOffset,
      false);

  // Field visualization and gyro
  public final Field2d m_field = new Field2d();
  public static AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  public static double robotAngleSim = 0;

  // Slew rate limiters for smooth control
  private SlewRateLimiter m_magXLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_magYLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration);

  static VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static FieldPositions fieldPositions = new FieldPositions();

  private double kP = 3;
  private double kI = 0;
  private double kD = 0;

  /*
   * Constructs the DriveSubsystem and configures autonomous settings.
   */
  public DriveSubsystem() {
    configureAutoBuilder();
  }

  public void configureAutoBuilder() {
    try {
      AutoBuilder.configure(this::getPose, this::resetOdometry, this::getSpeeds, this::setSpeeds,
          new PPHolonomicDriveController(
              new PIDConstants(kP, kI, kD),
              new PIDConstants(3, 0, 0.0)),
          RobotConfig.fromGUISettings(), () -> !isAllianceBlue(),
          this);

    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), true);
    }
  }

  /**
   * Retrieves the current speeds of the robot.
   * 
   * @return The chassis speeds.
   */
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    robotAngleSim += speeds.omegaRadiansPerSecond * 2;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    m_frontLeft.updateSmartDashboard();
    // Simulation only

    // m_rearLeft.updateMotorPosition(1.5);
    // m_rearRight.updateMotorPosition(1.5);
    // m_frontLeft.updateMotorPosition(1.5);
    // m_frontRight.updateMotorPosition(1.5);

    m_field.setRobotPose(getPose());

    SmartDashboard.putData(m_field);
  }

  /**
   * Retrieves the current pose of the robot.
   * 
   * @return The robot's pose.
   */
  public Pose2d getPose() {
    return visionSubsystem.GetRobotPoseEstimated();
  }

  public static Pose3d getRPose3d() {
    return new Pose3d(visionSubsystem.GetRobotPoseEstimated());
  }

  /**
   * Gets the field visualization object.
   * 
   * @return The field visualization.
   */
  public Field2d getField() {
    return m_field;
  }

  /**
   * Resets the robot's odometry to a specific pose.
   * 
   * @param pose The new pose.
   */
  public void resetOdometry(Pose2d pose) {
    visionSubsystem.resetOdometry(pose);
  }

  /**
   * Drives the robot with the given speeds and modes.
   * 
   * @param xSpeed       Speed in the X direction.
   * @param ySpeed       Speed in the Y direction.
   * @param rot          Rotational speed.
   * @param robotCentric Whether the driving is robot-centric.
   * @param slowSpeed    Whether to use slow speed mode.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean slowSpeed,
      boolean isJoystick) {
    double xSpeedDelivered, ySpeedDelivered, rotDelivered;

    robotAngleSim += rot * 2;

    if (slowSpeed) {
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed * 0.3) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed * 0.3) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot * 0.5) * DriveConstants.kMaxAngularSpeed;
    } else {
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getHeading().plus(Rotation2d.fromDegrees(!isAllianceBlue() ? 180 : 0)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the states for the swerve modules.
   * 
   * @param desiredStates The desired states for each swerve module.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public static void lockSwerve() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  PathConstraints constraints = new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecondPathfind,
      DriveConstants.kMaxAccelerationPathfind,
      DriveConstants.kMaxAngularSpeedPathfind,
      DriveConstants.kMaxAngularAccelerationPathfind);

  public Command goToPosePathfind(PathfindType type, boolean right) {
    return Commands.runOnce(() -> {
      if (LimelightHelpers.getTargetCount("") < 1 && type == PathfindType.Reef)
        return;

      Pose2d target;

      if (type == PathfindType.Reef)
        target = fieldPositions.getRightLeftReef(fieldPositions.getClosestTag(getPose()), right,
            () -> isAllianceBlue());
      else if (type == PathfindType.Human)
        target = fieldPositions.getClosestHumanPose(getPose());
      else if (type == PathfindType.Algea)
        target = fieldPositions.getClosestAlgeaPose(getPose());
      else
        target = fieldPositions.getPose("Processor");

      if (target == null)
        return;

      AutoBuilder.pathfindToPose(target, constraints).schedule();
    });
  }

  public Command goToPosePathfind(String namString) {
    return Commands.runOnce(() -> {
      Pose2d target;

      target = fieldPositions.getPose(namString);

      if (target == null)
        return;

      AutoBuilder.pathfindToPose(target, constraints).schedule();
    });
  }

  boolean movedOnce = false;

  public boolean finishedPath() {
    if (!movedOnce && Math.abs(getSpeeds().vyMetersPerSecond) > 0.5 && Math.abs(getSpeeds().vxMetersPerSecond) > 0.5)
      movedOnce = true;

    if (Math.abs(getSpeeds().vyMetersPerSecond) < 0.1 && movedOnce && Math.abs(getSpeeds().vxMetersPerSecond) < 0.1) {
      movedOnce = false;
      return true;
    }

    return false;
  }

  /**
   * Resets the gyro heading to zero.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public static boolean isAllianceBlue() {
    return DriverStation.getAlliance().isEmpty() ? true : DriverStation.getAlliance().get() == Alliance.Blue;
  }

  /**
   * Retrieves the current heading of the robot.
   * 
   * @return The current heading as a Rotation2d.
   */
  public static Rotation2d getHeading() {
    if (Robot.isSimulation())
      return Rotation2d.fromDegrees(robotAngleSim)
          .plus(Rotation2d.fromDegrees(isAllianceBlue() ? 180 : 0));
    ;
    return m_gyro.getRotation2d()
        .plus(Rotation2d.fromDegrees(isAllianceBlue() ? 180 : 0));
  }

  public static SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = m_frontRight.getPosition();
    positions[1] = m_frontLeft.getPosition();
    positions[2] = m_rearRight.getPosition();
    positions[3] = m_rearLeft.getPosition();
    return positions;
  }

  /**
   * Resets the encoders for all swerve modules.
   */
  public static void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
    m_gyro.reset();
  }

  /**
   * Resets the swerve modules to their absolute encoder positions.
   */
  public static void resetToAbsolute() {
    m_frontLeft.resetToAbsolute();
    m_frontRight.resetToAbsolute();
    m_rearLeft.resetToAbsolute();
    m_rearRight.resetToAbsolute();
    m_gyro.reset();
  }

  /**
   * Stops all swerve modules.
   */
  public void stop() {
    m_rearLeft.stop();
    m_frontLeft.stop();
    m_rearRight.stop();
    m_frontRight.stop();
  }
}