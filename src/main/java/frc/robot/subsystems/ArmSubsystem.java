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
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.misc.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
    // Motor responsible for rotating the arm
    public static final SparkMax armMotor = new SparkMax(ArmConstants.kArmMotorCanID, MotorType.kBrushless);
    SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();
    private double armGoalPosition = 0;

    /**
     * Constructor that configures the arm motor settings.
     */
    public ArmSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.closedLoop.pid(ArmConstants.kArmMotorP, ArmConstants.kArmMotorI, ArmConstants.kArmMotorD);
        armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        armMotor.getEncoder().setPosition(0);
    }

    /**
     * Creates a command to move the arm to a specific position.
     *
     * @param positionSelection The desired arm position.
     * @return The command to set the arm position.
     */
    public Command setArmPositionCommand(ArmPosition positionSelection) {
        return new Command() {
            @Override
            public void initialize() {
                armGoalPosition = getArmPositionValue(positionSelection);
                setArmPosition(getArmPositionValue(positionSelection));
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
        SmartDashboard.putNumber("Arm/Arm Position", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm/Arm Goal", armGoalPosition);
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
                return RobotContainer.coralMode ? -20.0 : -7000;
            case grab_algae_reef_1:
                return -7000.0;
            case grab_algae_reef_2:
                return -7000.0;
            case drop_algae_net:
                return -2000.0;
            case place_algae_processor:
                return -7000.0;
            case place_coral_l1:
                return -20.0;
            case place_coral_l2:
                return -20.0;
            case ElevatorUp:
                return -200.0;
            case ElevatorUpAlgae:
                return -7200.0;
            case place_coral_l3:
                return -20.0;
            case place_coral_l4:
                return -20.0;
            default:
                return -20.0;
        }
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
        return MathUtil.isNear(armGoalPosition, armMotor.getEncoder().getPosition(), 100);
    }
}