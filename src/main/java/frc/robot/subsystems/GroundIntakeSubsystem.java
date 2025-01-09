package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final TalonFX groundIntakeRotationMotor = new TalonFX(
            GroundIntakeConstants.kGroundIntakeRotationMotorCanId);
    private final SparkMax groundIntakeFeedMotor = new SparkMax(GroundIntakeConstants.kGroundIntakeFeedMotorCanId,
            MotorType.kBrushless);

    private final DigitalInput groundIntakeSensor = new DigitalInput(GroundIntakeConstants.kGroundIntakeSensorPort);

    public GroundIntakeSubsystem() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.Slot0.kP = GroundIntakeConstants.kGroundIntakeRotationP;
        talonFXConfiguration.Slot0.kI = GroundIntakeConstants.kGroundIntakeRotationI;
        talonFXConfiguration.Slot0.kD = GroundIntakeConstants.kGroundIntakeRotationD;
        talonFXConfiguration.MotorOutput.PeakForwardDutyCycle = GroundIntakeConstants.kGroundIntakeRotationMaxSpeed;
        talonFXConfiguration.MotorOutput.PeakReverseDutyCycle = -GroundIntakeConstants.kGroundIntakeRotationMaxSpeed;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = GroundIntakeConstants.kGroundIntakeOpenedAngle;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = GroundIntakeConstants.kGroundIntakeClosedAngle;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = GroundIntakeConstants.kGroundIntakeRotationMotorReduction;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = GroundIntakeConstants.kGroundIntakeRotationMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        groundIntakeRotationMotor.getConfigurator().apply(talonFXConfiguration);

        SparkMaxConfig groundIntakeFeedMotorConfig = new SparkMaxConfig();
        groundIntakeFeedMotorConfig.idleMode(IdleMode.kBrake);
        groundIntakeFeedMotorConfig.inverted(GroundIntakeConstants.kGroundIntakeFeedMotorInverted);
        groundIntakeFeedMotor.configure(groundIntakeFeedMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command setIntakePositionCommand(boolean open) {
        return new Command() {
            @Override
            public void initialize() {
                setIntakePosition(open);
            }

            @Override
            public boolean isFinished() {
                return isIntakeReady(open);
            }
        };
    }

    public void setIntakePosition(boolean open) {
        groundIntakeRotationMotor.setControl(new PositionDutyCycle(open ? GroundIntakeConstants.kGroundIntakeOpenedAngle
        : GroundIntakeConstants.kGroundIntakeClosedAngle).withEnableFOC(true));
    }

    public boolean isIntakeReady(boolean goalOpen) {
        return MathUtil.isNear(
                goalOpen ? GroundIntakeConstants.kGroundIntakeOpenedAngle
                        : GroundIntakeConstants.kGroundIntakeClosedAngle,
                groundIntakeRotationMotor.getPosition().getValueAsDouble(),
                1);
    }

    public boolean isAlgeaDetected() {
        return !groundIntakeSensor.get();
    }

    public Command grabAlgea() {
        return new Command() {
            @Override
            public void execute() {
                groundIntakeFeedMotor.set(1);
            }

            @Override
            public void end(boolean interrupted) {
                groundIntakeFeedMotor.set(0);
            }

            @Override
            public boolean isFinished() {
                return isAlgeaDetected();
            }
        };
    }

    public Command stopMotors() {
        return new InstantCommand(() -> groundIntakeFeedMotor.set(0));
    }

    public Command spitAlgea() {
        return new Command() {
            @Override
            public void execute() {
                groundIntakeFeedMotor.set(1);
            }

            @Override
            public void end(boolean interrupted) {
                groundIntakeFeedMotor.set(0);
            }

            @Override
            public boolean isFinished() {
                return !AlgaeIntake.AlgaeArmIntakeSensor.get();
            }
        };
    }
}