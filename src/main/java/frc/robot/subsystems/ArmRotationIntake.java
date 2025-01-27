/**
 * This class manages the arm rotation for handling various intake operations.
 * It controls the arm motor, its position, and provides commands for movements.
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.misc.ArmPosition;

public class ArmRotationIntake extends SubsystemBase {
    // Motor responsible for rotating the arm
    private final TalonFX armMotor = new TalonFX(ElevatorConstants.kElevatorMotorCanID);
    private double armGoalPosition = 0;

    /**
     * Constructor that configures the arm motor settings.
     */
    public ArmRotationIntake() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = ArmConstants.kArmMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.kArmMotorSensorToMechRatio;
        talonFXConfiguration.Slot0.kP = ArmConstants.kArmMotorP;
        talonFXConfiguration.Slot0.kI = ArmConstants.kArmMotorI;
        talonFXConfiguration.Slot0.kD = ArmConstants.kArmMotorD;
        talonFXConfiguration.MotorOutput.PeakForwardDutyCycle = ArmConstants.kArmMotorMaxSpeed;
        talonFXConfiguration.MotorOutput.PeakReverseDutyCycle = -ArmConstants.kArmMotorMaxSpeed;
        armMotor.getConfigurator().apply(talonFXConfiguration);
    }

    /**
     * Creates a command to move the arm to a specific position.
     *
     * @param positionSelection The desired arm position.
     * @return The command to set the arm position.
     */
    public Command setArmPositionCommand(ArmPosition positionSelection) {
        armGoalPosition = getArmPositionValue(positionSelection);
        return new Command() {
            @Override
            public void initialize() {
                setArmPosition(armGoalPosition);
            }

            @Override
            public boolean isFinished() {
                return isArmAtPosition();
            }
        };
    }

    /**
     * Updates the SmartDashboard with the arm's current and goal positions and temperature.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Goal", armGoalPosition);
        SmartDashboard.putNumber("Arm temp", armMotor.getDeviceTemp().getValueAsDouble());
    }

    /**
     * Retrieves the numerical value of a specific arm position.
     *
     * @param position The desired arm position.
     * @return The corresponding numerical value.
     */
    private double getArmPositionValue(ArmPosition position) {
        switch (position) {
            case idle:
                return RobotContainer.coralMode ? 0 : 180;
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

    /**
     * Sets the arm's position to a specific value.
     *
     * @param position The desired position value.
     */
    public void setArmPosition(double position) {
        armMotor.setControl(new PositionDutyCycle(position).withEnableFOC(true));
    }

    /**
     * Sets the arm's position to a predefined ArmPosition.
     *
     * @param position The ArmPosition to set.
     */
    public void setArmPosition(ArmPosition position) {
        armMotor.setControl(new PositionDutyCycle(getArmPositionValue(position)).withEnableFOC(true));
    }

    /**
     * Checks if the arm has reached its goal position.
     *
     * @return True if the arm is at the goal position, false otherwise.
     */
    public boolean isArmAtPosition() {
        return MathUtil.isNear(armGoalPosition, armMotor.getPosition().getValueAsDouble(), 10);
    }
}