package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.misc.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    public final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorCanID);
    public final TalonFX elevatorMotor2 = new TalonFX(ElevatorConstants.kElevatorMotorCanID2);

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
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kElevatorMotorCruiseVelocity
                / 60;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ElevatorConstants.kElevatorMotorSensorToMechRatio;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = ElevatorConstants.kElevatorMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        elevatorMotor.getConfigurator().apply(talonFXConfiguration);
        elevatorMotor2.setControl(new Follower(elevatorMotor.getDeviceID(), false));
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

    /**
     * Updates the SmartDashboard with the elevator's current and goal positions and temperature.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Goal", elevatorGoalPosition);
        SmartDashboard.putNumber("Elevator temp", elevatorMotor.getDeviceTemp().getValueAsDouble());
    }

    private double getElevatorPositionValue(ElevatorPosition position) {
        switch (position) {
            case idle:
                return 90;
            case grab_algae_reef_1:
                return 90;
            case grab_algae_reef_2:
                return 0;
            case grab_coral_source:
                return 0;
            case place_algae_processor:
                return 90;
            case place_coral_l1:
                return 0;
            case place_coral_l2:
                return 0;
            case place_coral_l3:
                return 90;
            case place_coral_l4:
                return 90;
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