package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.pathfind.FieldPositions;
import frc.robot.subsystems.pathfind.PathfindType;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The DriveSubsystem class manages the swerve drive system of the robot.
 * It controls the four swerve modules and handles field-relative and
 * robot-relative driving.
 */
public class DriveSubsystem extends SubsystemBase {

  // Swerve modules for each corner of the robot
  public static final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftcanCoderIDCanId,
      true,
      true,
      DriveConstants.kFrontLeftcanCoderOffset,
      true);

  public static final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightcanCoderIDCanId,
      true,
      true,
      DriveConstants.kFrontRightcanCoderOffset,
      true);

  public static final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftcanCoderIDCanId,
      true,
      true,
      DriveConstants.kRearLeftcanCoderOffset,
      true);

  public static final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightcanCoderIDCanId,
      true,
      true,
      DriveConstants.kRearRightcanCoderOffset,
      true);

  private final StructArrayPublisher<SwerveModuleState> publisher;

  // Field visualization and gyro
  public final Field2d m_field = new Field2d();
  public static AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Slew rate limiters for smooth control
  private SlewRateLimiter m_magXLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_magYLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration);

  static VisionSubsystem visionSubsystem = new VisionSubsystem();
  FieldPositions fieldPositions = new FieldPositions();

  /**
   * Constructs the DriveSubsystem and configures autonomous settings.
   */
  public DriveSubsystem() {
    try {
      AutoBuilder.configure(this::getPose, this::resetOdometry, this::getSpeeds, this::setSpeeds,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          RobotConfig.fromGUISettings(), () -> {
            var alliance = DriverStation.getAlliance().get();
            return alliance == DriverStation.Alliance.Red;
          },
          this);
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), true);
    }

    publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
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

  /**
   * Sets the desired speeds for the robot.
   * 
   * @param speeds The desired chassis speeds.
   */
  public void setSpeeds(ChassisSpeeds speeds) {
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
    m_rearLeft.updateSmartDashboard();
    m_rearRight.updateSmartDashboard();
    m_frontLeft.updateSmartDashboard();
    m_frontRight.updateSmartDashboard();

    SmartDashboard.putData(m_gyro);
    SmartDashboard.putData(m_field);

    // Periodically send a set of module states
    publisher.set(new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    });
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean robotCentric, boolean slowSpeed) {
    double xSpeedDelivered, ySpeedDelivered, rotDelivered;

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
        robotCentric
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getHeading().plus(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180)))
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

  public Pose2d getClosestPose(Pose2d... poses) {
        Pose2d closestPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (Pose2d pose2d: poses) {
          double distance = PhotonUtils.getDistanceToPose(getPose(), pose2d);
          if (distance < closestDistance) {
              closestDistance = distance;
              closestPose = pose2d;
          }
        }
        return closestPose;
    }

  public Command goToPosePathfind(PathfindType pathfindType) {
    Pose2d pose = new Pose2d();
    switch (pathfindType) {
      case Reef:
        pose = fieldPositions.getClosestReefPose(getPose());
        break;

      case Human:
        pose = fieldPositions.getClosestHumanPose(getPose());
        break;

      case Algea:
        pose = fieldPositions.getPose("algea");
        break;

      case Processor:
        pose = fieldPositions.getPose("processor");
        break;
      
      case Closest:
        pose = getClosestPose(fieldPositions.getPose("processor"), fieldPositions.getClosestHumanPose(getPose()), fieldPositions.getClosestReefPose(getPose()));
        break;

      default:
        break;
    }

    try {
      if(pose.getX() == 0)
      {
        throw new Exception("No valid pose found");
      }

      PathConstraints constraints = new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecondPathfind,
          DriveConstants.kMaxAccelerationPathfind,
          DriveConstants.kMaxAngularSpeedPathfind,
          DriveConstants.kMaxAngularAccelerationPathfind);

      return AutoBuilder.pathfindToPose(pose, constraints, 0);
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), true);
      return null;
    }
  }

  /**
   * Resets the gyro heading to zero.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Retrieves the current heading of the robot.
   * 
   * @return The current heading as a Rotation2d.
   */
  public static Rotation2d getHeading() {
    return m_gyro.getRotation2d();
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