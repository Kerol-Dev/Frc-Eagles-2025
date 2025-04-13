package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    public static final TalonFX elevatorMotor2 = new TalonFX(ElevatorConstants.kElevatorMotorCanID2);

    private double elevatorGoalPosition = 0;

    public static void setElevatorConfiguration(boolean isCoral) {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        // set slot 0 gains
        talonFXConfiguration.Slot0.kP = ElevatorConstants.kElevatorMotorP;
        talonFXConfiguration.Slot0.kI = ElevatorConstants.kElevatorMotorI;
        talonFXConfiguration.Slot0.kD = ElevatorConstants.kElevatorMotorD;
        talonFXConfiguration.MotorOutput.PeakForwardDutyCycle = ElevatorConstants.kElevatorMaxSpeed;
        talonFXConfiguration.MotorOutput.PeakReverseDutyCycle = -ElevatorConstants.kElevatorMaxSpeedDown
                / (isCoral ? 1 : 4);
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kElevatorMotorForwardSoftLimit;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.kElevatorMotorReverseSoftLimit;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ElevatorConstants.kElevatorMotorSensorToMechRatio;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // elevatorMotor.getConfigurator().apply(talonFXConfiguration);
        // elevatorMotor2.getConfigurator().apply(talonFXConfiguration);
    }

    public ElevatorSubsystem() {
        setElevatorConfiguration(true);
        elevatorMotor.setPosition(0);
        elevatorMotor2.setPosition(0);
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
     * Updates the SmartDashboard with the elevator's current and goal positions and
     * temperature.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Elevator Goal", elevatorGoalPosition);
    }

    private double getElevatorPositionValue(ElevatorPosition position) {
        switch (position) {
            case idle:
                return RobotContainer.coralMode ? 0.0 : 0.5;
            case grab_algae_reef_1:
                return 1.6;
            case grab_algae_reef_2:
                return 2.9;
            case place_algae_net:
                return 4.18;
            case place_algae_processor:
                return 0.4;
            case place_coral_l1:
                return 0.5;
            case place_coral_l2:
                return 1;
            case place_coral_l3:
                return 2.31;
            case place_coral_l4:
                return 4.18;
            default:
                return 0.0;
        }
    }

    public void setElevatorPosition(double position) {
        elevatorGoalPosition = position;
        elevatorMotor.setControl(new PositionDutyCycle(position));
        elevatorMotor2.setControl(new PositionDutyCycle(position));
    }

    public void setElevatorPosition(ElevatorPosition position) {
        elevatorMotor.setControl(new PositionDutyCycle(getElevatorPositionValue(position)));
        elevatorMotor2.setControl(new PositionDutyCycle(getElevatorPositionValue(position)));
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(elevatorGoalPosition, elevatorMotor.getPosition().getValueAsDouble(), 0.15);
    }
}