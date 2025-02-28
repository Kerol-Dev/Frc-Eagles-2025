package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.misc.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    public static final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorCanID);
    public final TalonFX elevatorMotor2 = new TalonFX(ElevatorConstants.kElevatorMotorCanID2);

    private double elevatorGoalPosition = 0;

    public ElevatorSubsystem() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        // set slot 0 gains
        talonFXConfiguration.Slot0.kP = ElevatorConstants.kElevatorMotorP;
        talonFXConfiguration.Slot0.kI = ElevatorConstants.kElevatorMotorI;
        talonFXConfiguration.Slot0.kD = ElevatorConstants.kElevatorMotorD;
        talonFXConfiguration.MotorOutput.PeakForwardDutyCycle = ElevatorConstants.kElevatorMaxSpeed;
        talonFXConfiguration.MotorOutput.PeakReverseDutyCycle = -ElevatorConstants.kElevatorMaxSpeedDown;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kElevatorMotorForwardSoftLimit;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.kElevatorMotorReverseSoftLimit;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ElevatorConstants.kElevatorMotorSensorToMechRatio;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorMotor.getConfigurator().apply(talonFXConfiguration);
        elevatorMotor2.setControl(new Follower(elevatorMotor.getDeviceID(), false));

        elevatorMotor.setPosition(0);
    }

    public Command setElevatorPositionCommand(ElevatorPosition positionSelection) {
        return new Command() {
            @Override
            public void initialize() {
                elevatorGoalPosition = getElevatorPositionValue(positionSelection);
                setElevatorPosition(getElevatorPositionValue(positionSelection));
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
                return RobotContainer.coralMode ? 0.05 : 0.4;
            case grab_algae_reef_1:
                return 1.5;
            case grab_algae_reef_2:
                return 2.9;
            case place_algae_processor:
                return 0.4;
            case place_coral_l:
                return 0.5;
            case place_coral_l2:
                return 1.01;
            case place_coral_l3:
                return 2.22;
            case place_coral_l4:
                return 4.31;
            default:
                return 0.0;
        }
    }

    public void setElevatorPosition(double position) {
        elevatorGoalPosition = position;
        elevatorMotor.setControl(new PositionDutyCycle(position));
    }

    public void setElevatorPosition(ElevatorPosition position) {
        elevatorMotor.setControl(new PositionDutyCycle(getElevatorPositionValue(position)));
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(elevatorGoalPosition, elevatorMotor.getPosition().getValueAsDouble(), 0.3);
    }
}