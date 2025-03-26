package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * Represents a single swerve module with driving and turning capabilities.
 * This version uses simulated motors instead of real motors.
 */
@AutoLog
public class SimulatedSwerveModule {

  // Simulated motor controllers for driving and turning
  private double m_drivingMotorPosition = 0.0;
  private double m_turningMotorPosition = 0.0;
  private double m_drivingMotorVelocity = 0.0;

  // Offset values for encoder calibration
  private double m_chassisAngularOffset = 0;
  public double encoderOffset;
  private Rotation2d encoderOffset2d;

  // Desired state of the swerve module
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SimulatedSwerveModule.
   * 
   * @param encoderOffset Offset for the encoder
   */
  public SimulatedSwerveModule(double encoderOffset) {
    encoderOffset2d = Rotation2d.fromDegrees(encoderOffset);
    this.encoderOffset = encoderOffset;
    m_desiredState.angle = new Rotation2d(0);
  }

  /**
   * Gets the current state of the swerve module.
   * 
   * @return The current state of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingMotorVelocity, getAngle());
  }

  /**
   * Gets the current position of the swerve module.
   * 
   * @return The current position of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getMotorPosition(), getAngle());
  }

  /**
   * Gets the current angle of the swerve module.
   * 
   * @return The current angle
   */
  private Rotation2d getAngle() {
    return Rotation2d.fromRadians(m_turningMotorPosition - m_chassisAngularOffset);
  }

  /**
   * Gets the position of the driving motor.
   * 
   * @return The position of the motor
   */
  private double getMotorPosition() {
    return -m_drivingMotorPosition * ModuleConstants.kDrivingEncoderPositionFactor;
  }

  /**
   * Updates the SmartDashboard with encoder information.
   */
  public void updateSmartDashboard() {
    if(encoderOffset > 0)
    {
        SmartDashboard.putNumber("Simulated Position", m_drivingMotorPosition);
        SmartDashboard.putNumber("Simulated Turning Motor Position", m_turningMotorPosition);
    }
  }

  /**
   * Sets the desired state of the swerve module.
   * 
   * @param desiredState The desired state to set
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    SwerveModuleState optimizedDesiredState = optimize(correctedDesiredState,
        new Rotation2d(m_turningMotorPosition));

    m_drivingMotorVelocity = optimizedDesiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.006) {
      m_drivingMotorVelocity = 0;
    }

    m_turningMotorPosition = optimizedDesiredState.angle.getRadians();

    m_desiredState = desiredState;
  }

  /**
   * Updates the motor position based on the current velocity.
   * This method should be called periodically.
   */
  public void updateMotorPosition(double timeStep) {
    m_drivingMotorPosition += m_drivingMotorVelocity * timeStep;
  }

  /**
   * Resets the driving motor encoder.
   */
  public void resetEncoders() {
    m_drivingMotorPosition = 0;
  }

  /**
   * Resets the turning motor to the absolute encoder position.
   */
  public void resetToAbsolute() {
    double absolutePosition = encoderOffset2d.getRadians();
    m_turningMotorPosition = absolutePosition;
    resetEncoders();
  }

  /**
   * Stops the swerve module.
   */
  public void stop() {
    m_drivingMotorVelocity = 0;
    m_turningMotorPosition = 0;
  }

  /**
   * Optimizes the desired state based on the current angle to minimize rotation.
   * 
   * @param desiredState The desired state
   * @param currentAngle The current angle
   * @return The optimized state
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }
}
