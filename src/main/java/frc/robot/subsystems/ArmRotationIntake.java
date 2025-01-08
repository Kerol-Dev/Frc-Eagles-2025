package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
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
        // set slot 0 gains
        var slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        talonFXConfiguration.Slot0.kP = ArmConstants.kArmMotorP;
        talonFXConfiguration.Slot0.kI = ArmConstants.kArmMotorI;
        talonFXConfiguration.Slot0.kD = ArmConstants.kArmMotorD;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.kArmMotorForwardSoftLimit;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.kArmMotorReverseSoftLimit;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = ArmConstants.kArmMotorAcceleration;
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kArmMotorCruiseVelocity;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.kArmMotorSensorToMechRatio;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput.Inverted = ArmConstants.kArmMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

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

    private double getArmPositionValue(ArmPosition position)
    {
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
        armMotor.setControl(new MotionMagicVoltage(position).withEnableFOC(true));
    }

    public void setArmPosition(ArmPosition position) {
        armMotor.setControl(new MotionMagicVoltage(getArmPositionValue(position)).withEnableFOC(true));
    }

    public boolean isArmAtPosition() {
        return MathUtil.isNear(armGoalPosition, armMotor.getPosition().getValueAsDouble(), 10);
    }
}