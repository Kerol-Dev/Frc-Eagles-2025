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
        sparkMaxConfig.encoder
                .positionConversionFactor(ArmConstants.kArmMotorSensorToMechRatio);
        sparkMaxConfig.closedLoop.pid(ArmConstants.kArmMotorP,
                ArmConstants.kArmMotorI, ArmConstants.kArmMotorD);
        sparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
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

    private double getArmPositionValue(ArmPosition position) {
        switch (position) {
            case coral_L4:
                return 150;
            case coral_L23:
                return 135;
            case coral_L1:
                return 90;
            case idle:
                return 0;
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