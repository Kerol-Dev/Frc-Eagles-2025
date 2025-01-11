package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.misc.ArmPosition;

public class ArmRotationIntake extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(ElevatorConstants.kElevatorMotorCanID, MotorType.kBrushless);
    private double armGoalPosition = 0;

    public ArmRotationIntake() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(ArmConstants.kArmMotorInverted);
        // sparkMaxConfig.encoder
        //         .positionConversionFactor(ArmConstants.kArmMotorSensorToMechRatio);
        sparkMaxConfig.closedLoop.pid(ArmConstants.kArmMotorP,
                ArmConstants.kArmMotorI, ArmConstants.kArmMotorD);
        sparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        sparkMaxConfig.closedLoop.outputRange(-ArmConstants.kArmMotorMaxSpeed,
                ArmConstants.kArmMotorMaxSpeed);
        armMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Arm Goal", armGoalPosition);
        SmartDashboard.putNumber("Arm temp", armMotor.getMotorTemperature());
    }

    private double getArmPositionValue(ArmPosition position) {
        switch (position) {
            case idle:
                return 0;
            case grab_algae_ground:
                return 0;
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

    public void setArmPosition(double position) {
        armMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    public void setArmPosition(ArmPosition position) {
        armMotor.getClosedLoopController().setReference(getArmPositionValue(position), ControlType.kPosition);
    }

    public boolean isArmAtPosition() {
        return MathUtil.isNear(armGoalPosition, armMotor.getEncoder().getPosition(), 10);
    }
}