package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final SparkMax groundIntakeRotationMotor_1 = new SparkMax(
            GroundIntakeConstants.kGroundIntakeRotationMotor1CanId, MotorType.kBrushless);
    private final SparkMax groundIntakeRotationMotor_2 = new SparkMax(
            GroundIntakeConstants.kGroundIntakeRotationMotor2CanId, MotorType.kBrushless);
    private final SparkMax groundIntakeFeedMotor = new SparkMax(GroundIntakeConstants.kGroundIntakeFeedMotorCanId,
            MotorType.kBrushless);

    private final DigitalInput groundIntakeSensor = new DigitalInput(GroundIntakeConstants.kGroundIntakeSensorPort);

    public GroundIntakeSubsystem() {
        SparkMaxConfig groundIntakeRotationMotor1Config = new SparkMaxConfig();
        groundIntakeRotationMotor1Config.idleMode(IdleMode.kBrake);
        groundIntakeRotationMotor1Config.inverted(GroundIntakeConstants.kGroundIntakeRotationMotor1Inverted);
        groundIntakeRotationMotor1Config.follow(groundIntakeRotationMotor_2);
        groundIntakeRotationMotor_1.configure(groundIntakeRotationMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SparkMaxConfig groundIntakeRotationMotor2Config = new SparkMaxConfig();
        groundIntakeRotationMotor2Config.idleMode(IdleMode.kBrake);
        groundIntakeRotationMotor2Config.inverted(GroundIntakeConstants.kGroundIntakeRotationMotor1Inverted);
        groundIntakeRotationMotor2Config.encoder
                .positionConversionFactor(360 / GroundIntakeConstants.kGroundIntakeRotationMotor2Reduction);
        groundIntakeRotationMotor2Config.closedLoop.pid(GroundIntakeConstants.kGroundIntakeRotationP,
                GroundIntakeConstants.kGroundIntakeRotationI, GroundIntakeConstants.kGroundIntakeRotationD);
        groundIntakeRotationMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        groundIntakeRotationMotor2Config.closedLoop.outputRange(-GroundIntakeConstants.kGroundIntakeRotationMaxSpeed,
                GroundIntakeConstants.kGroundIntakeRotationMaxSpeed);
        groundIntakeRotationMotor_2.configure(groundIntakeRotationMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SparkMaxConfig groundIntakeFeedMotorConfig = new SparkMaxConfig();
        groundIntakeFeedMotorConfig.idleMode(IdleMode.kBrake);
        groundIntakeFeedMotorConfig.inverted(GroundIntakeConstants.kGroundIntakeRotationMotor1Inverted);
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
        groundIntakeRotationMotor_2.getClosedLoopController().setReference(
                open ? GroundIntakeConstants.kGroundIntakeClosedAngle : GroundIntakeConstants.kGroundIntakeOpenedAngle,
                ControlType.kPosition);
    }

    public boolean isIntakeReady(boolean goalOpen) {
        return MathUtil.isNear(
                goalOpen ? GroundIntakeConstants.kGroundIntakeOpenedAngle
                        : GroundIntakeConstants.kGroundIntakeClosedAngle,
                groundIntakeRotationMotor_2.getEncoder().getPosition(),
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
                return !AlgaeArmIntake.AlgaeArmIntakeSensor.get();
            }
        };
    }
}