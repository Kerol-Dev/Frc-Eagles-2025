package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.misc.ArmPosition;

public class ArmRotationIntake extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(ElevatorConstants.kElevatorMotorCanID);
    private double armGoalPosition = 0;

    public ArmRotationIntake() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = ArmConstants.kArmMotorInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.kArmMotorSensorToMechRatio;
        talonFXConfiguration.Slot0.kP = ArmConstants.kArmMotorP;
        talonFXConfiguration.Slot0.kI = ArmConstants.kArmMotorI;
        talonFXConfiguration.Slot0.kD = ArmConstants.kArmMotorD;
        talonFXConfiguration.MotorOutput.PeakForwardDutyCycle = ArmConstants.kArmMotorMaxSpeed;
        talonFXConfiguration.MotorOutput.PeakReverseDutyCycle = -ArmConstants.kArmMotorMaxSpeed;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.kArmMotorForwardSoftLimit;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.kArmMotorReverseSoftLimit;
        armMotor.getConfigurator().apply(talonFXConfiguration);
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
        SmartDashboard.putNumber("Arm Position", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Goal", armGoalPosition);
        SmartDashboard.putNumber("Arm temp", armMotor.getDeviceTemp().getValueAsDouble());
    }

    private double getArmPositionValue(ArmPosition position) {
        switch (position) {
            case idle:
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
        armMotor.setControl(new PositionDutyCycle(position).withEnableFOC(true));
    }

    public void setArmPosition(ArmPosition position) {
        armMotor.setControl(new PositionDutyCycle(getArmPositionValue(position)).withEnableFOC(true));
    }

    public boolean isArmAtPosition() {
        return MathUtil.isNear(armGoalPosition, armMotor.getPosition().getValueAsDouble(), 10);
    }
}