package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.misc.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    public final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorCanID);
    private double elevatorGoalPosition = 0;

    public ElevatorSubsystem() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        talonFXConfiguration.Slot0.kP = ElevatorConstants.kElevatorMotorP;
        talonFXConfiguration.Slot0.kI = ElevatorConstants.kElevatorMotorI;
        talonFXConfiguration.Slot0.kD = ElevatorConstants.kElevatorMotorD;
        talonFXConfiguration.MotorOutput.PeakForwardDutyCycle = ElevatorConstants.kElevatorMaxSpeed;
        talonFXConfiguration.MotorOutput.PeakReverseDutyCycle = -ElevatorConstants.kElevatorMaxSpeed;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kElevatorMotorForwardSoftLimit;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.kElevatorMotorReverseSoftLimit;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorMotorAcceleration / 60;
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kElevatorMotorCruiseVelocity / 60;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ElevatorConstants.kElevatorMotorSensorToMechRatio;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = ElevatorConstants.kElevatorMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        elevatorMotor.getConfigurator().apply(talonFXConfiguration);
    }

    public Command setElevatorPositionCommand(ElevatorPosition positionSelection) {
        elevatorGoalPosition = getElevatorPositionValue(positionSelection);
        return new Command() {
            @Override
            public void initialize() {
                setElevatorPosition(elevatorGoalPosition);
            }

            @Override
            public boolean isFinished() {
                return isElevatorAtPosition();
            }
        };
    }

    private double getElevatorPositionValue(ElevatorPosition position) {
        switch (position) {
            case coral_L4:
                return 0;
            case coral_L3:
                return 50;
            case coral_L2:
                return 70;
            case coral_L1:
                return 90;
            case processor:
                return 110;
            case idle:
                return 130;
            default:
                return 0;
        }
    }

    public void setElevatorPosition(double position) {
        elevatorMotor.setControl(new MotionMagicDutyCycle(position).withEnableFOC(true));
    }

    public void setElevatorPosition(ElevatorPosition position) {
        elevatorMotor.setControl(new MotionMagicDutyCycle(getElevatorPositionValue(position)).withEnableFOC(true));
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(elevatorGoalPosition, elevatorMotor.getPosition().getValueAsDouble(), 10);
    }
}