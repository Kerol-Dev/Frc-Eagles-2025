/**
 * This class manages the arm rotation for handling various intake operations.
 * It controls the arm motor, its position, and provides commands for movements.
 */
package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.misc.ArmPosition;

public class Pivot extends SubsystemBase {
    // Motor responsible for rotating the arm
    private final SparkMax armMotor = new SparkMax(ElevatorConstants.kElevatorMotorCanID, MotorType.kBrushless);
    SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();
    private double armGoalPosition = 0;

    /**
     * Constructor that configures the arm motor settings.
     */
    public Pivot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(ArmConstants.kArmMotorInverted);
        config.idleMode(IdleMode.kBrake);
        config.closedLoop.pid(ArmConstants.kArmMotorP, ArmConstants.kArmMotorI, ArmConstants.kArmMotorD);
        config.encoder.positionConversionFactor(ArmConstants.kArmMotorSensorToMechRatio);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Goal", armGoalPosition);
        SmartDashboard.putNumber("Arm temp", armMotor.getMotorTemperature());
    }

    /**
     * Retrieves the numerical value of a specific arm position.
     *
     * @param position The desired arm position.
     * @return The corresponding numerical value.
     */
    private double getArmPositionValue(ArmPosition position) {
        return position.getPosition();
    }

    /**
     * Sets the arm's position to a specific value.
     *
     * @param position The desired position value.
     */
    public void setArmPosition(double position) {
        armMotorController.setReference(position, ControlType.kPosition);
    }

    /**
     * Sets the arm's position to a predefined ArmPosition.
     *
     * @param position The ArmPosition to set.
     */
    public void setArmPosition(ArmPosition position) {
        armMotorController.setReference(getArmPositionValue(position), ControlType.kPosition);
    }

    /**
     * Checks if the arm has reached its goal position.
     *
     * @return True if the arm is at the goal position, false otherwise.
     */
    public boolean isArmAtPosition() {
        return MathUtil.isNear(armGoalPosition, armMotor.getEncoder().getPosition(), 10);
    }
}