package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * Represents a single swerve module with driving and turning capabilities.
 */
public class SwerveModule {

  // Motor controllers for driving and turning
  public final TalonFX m_drivingMotor;
  public final SparkMax m_turningSparkMax;

  // CANcoder for absolute encoder feedback
  public final CANcoder m_canEncoder;

  // Offset values for encoder calibration
  private double m_chassisAngularOffset = 0;
  public double encoderOffset;
  private Rotation2d encoderOffset2d;

  // Desired state of the swerve module
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule.
   * 
   * @param drivingCANId         CAN ID for the driving motor
   * @param turningCANId         CAN ID for the turning motor
   * @param cancoderID           CAN ID for the absolute encoder
   * @param drivingMotorReversed Whether the driving motor is reversed
   * @param turningMotorReversed Whether the turning motor is reversed
   * @param encoderOffset        Offset for the encoder
   * @param encoderInverted      Whether the encoder is inverted
   */
  public SwerveModule(int drivingCANId, int turningCANId, int cancoderID, boolean drivingMotorReversed,
      boolean turningMotorReversed, double encoderOffset, boolean encoderInverted) {

    encoderOffset2d = Rotation2d.fromDegrees(encoderOffset);
    this.encoderOffset = encoderOffset;

    // Configure the driving motor
    m_drivingMotor = new TalonFX(drivingCANId);
    m_drivingMotor.setNeutralMode(ModuleConstants.kDrivingMotorIdleMode);
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotorOutput.Inverted = drivingMotorReversed ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = ModuleConstants.kDrivingMotorCurrentLimit;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = ModuleConstants.kDrivingMotorStallCurrentLimit;
    m_drivingMotor.getConfigurator().apply(talonFXConfiguration);

    // Configure the turning motor
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // Configure the CANcoder
    m_canEncoder = new CANcoder(cancoderID);
    CANcoderConfiguration caNcoderConfiguration = new CANcoderConfiguration();
    caNcoderConfiguration.MagnetSensor.SensorDirection = encoderInverted ? SensorDirectionValue.Clockwise_Positive
        : SensorDirectionValue.CounterClockwise_Positive;
    m_canEncoder.getConfigurator().apply(caNcoderConfiguration);

    // Configure SparkMax for turning
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    config.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    config.closedLoop.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    config.closedLoop.p(ModuleConstants.kTurningP);
    config.closedLoop.i(ModuleConstants.kTurningI);
    config.closedLoop.d(ModuleConstants.kTurningD);
    config.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);
    config.idleMode(ModuleConstants.kTurningMotorIdleMode);
    config.smartCurrentLimit(ModuleConstants.kTurningMotorStallCurrentLimit, ModuleConstants.kTurningMotorCurrentLimit);
    m_turningSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_desiredState.angle = new Rotation2d(0);
  }

  /**
   * Gets the current state of the swerve module.
   * 
   * @return The current state of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingMotor.getVelocity().getValueAsDouble() * 60, getAngle());
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
    return Rotation2d.fromRadians(m_turningSparkMax.getEncoder().getPosition() - m_chassisAngularOffset);
  }

  /**
   * Gets the position of the driving motor.
   * 
   * @return The position of the motor
   */
  private double getMotorPosition() {
    return m_drivingMotor.getPosition().getValueAsDouble() * ModuleConstants.kDrivingEncoderPositionFactor;
  }

  /**
   * Updates the SmartDashboard with encoder information.
   */
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Cancoder " + m_canEncoder.getDeviceID(), getCanCoder().getDegrees());
    SmartDashboard.putNumber("NeoAngle " + m_canEncoder.getDeviceID(),
        Math.toDegrees((Math.abs(m_turningSparkMax.getEncoder().getPosition()) % (2.0 * Math.PI))));
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

    @SuppressWarnings("deprecation")
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningSparkMax.getEncoder().getPosition()));

    m_drivingMotor.set(optimizedDesiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.006)
      m_drivingMotor.stopMotor();
    m_turningSparkMax.getClosedLoopController().setReference(optimizedDesiredState.angle.getRadians(),
        SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /**
   * Gets the desired state of the swerve module.
   * 
   * @return The desired state
   */
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Resets the driving motor encoder.
   */
  public void resetEncoders() {
    m_drivingMotor.setPosition(0);
  }

  /**
   * Gets the CANcoder angle.
   * 
   * @return The CANcoder angle
   */
  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees((m_canEncoder.getAbsolutePosition().getValueAsDouble() * 360));
  }

  /**
   * Resets the turning motor to the absolute encoder position.
   */
  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRadians() - encoderOffset2d.getRadians();
    m_turningSparkMax.getEncoder().setPosition(absolutePosition);
    resetEncoders();
  }

  /**
   * Stops the swerve module.
   */
  public void stop() {
    m_drivingMotor.set(0);
    m_turningSparkMax.set(0);
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
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * Places the angle in the appropriate 0 to 360 scope relative to a reference.
   * 
   * @param scopeReference The reference angle
   * @param newAngle       The new angle to place
   * @return The adjusted angle
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}