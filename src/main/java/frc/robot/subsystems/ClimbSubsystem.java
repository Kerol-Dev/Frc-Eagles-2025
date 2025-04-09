/**
 * This class manages the arm rotation for handling various intake operations.
 * It controls the arm motor, its position, and provides commands for movements.
 */
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    // Motor responsible for rotating the arm
    private final SparkMax climbMotor = new SparkMax(ClimbConstants.kClimbMotorCanID, MotorType.kBrushless);
    SparkClosedLoopController climbMotorController = climbMotor.getClosedLoopController();
    Servo servo = new Servo(0);
    public static boolean climbEngaged = false;

    /**
     * Constructor that configures the arm motor settings.
     */
    public ClimbSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.closedLoop.pid(ClimbConstants.kClimbMotorP, ClimbConstants.kClimbMotorI, ClimbConstants.kClimbMotorD);
        config.softLimit.forwardSoftLimitEnabled(false);
        config.softLimit.forwardSoftLimit(ClimbConstants.kClimbMotorForwardSoftLimit);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.softLimit.reverseSoftLimit(ClimbConstants.kClimbMotorReverseSoftLimit);
        climbMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        climbMotor.getEncoder().setPosition(0);
    }

    /**
     * Creates a command to move the arm to a specific position.
     *
     * @param positionSelection The desired arm position.
     * @return The command to set the arm position.
     */
    public Command toggleClimbEngaged() {
        return new Command() {
            @Override
            public void initialize() {
                climbEngaged = !climbEngaged;
                setClimbPosition();
            }

            @Override
            public boolean isFinished() {
                return isClimbAtPosition();
            }
        };
    }

    public void setClimbSpeed(double speed) {
        climbMotor.set(speed);

        if(speed > 0 && DriverStation.isFMSAttached())
        {
            servo.setAngle(160);
        }
        else if(speed < 0)
        {
            servo.setAngle(90);
        }
    }

    public void stop() {
        climbMotor.stopMotor();
    }

    /**
     * Updates the SmartDashboard with the arm's current and goal positions and
     * temperature.
     */
    @Override
    public void periodic() {
        Logger.recordOutput("Climb/Climb Position", climbMotor.getEncoder().getPosition());
        Logger.recordOutput("Climb/Climb Engaged", climbEngaged);
    }

    /**
     * Sets the arm's position to a predefined ArmPosition.
     *
     * @param position The ArmPosition to set.
     */
    public void setClimbPosition() {
        climbMotorController.setReference(climbEngaged ? 400 : 0, ControlType.kPosition);
    }

    /**
     * Checks if the arm has reached its goal position.
     *
     * @return True if the arm is at the goal position, false otherwise.
     */
    public boolean isClimbAtPosition() {
        return MathUtil.isNear(climbEngaged ? 400 : 0, climbMotor.getEncoder().getPosition(), 30);
    }
}