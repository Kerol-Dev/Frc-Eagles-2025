package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmIntakeConstants;

public class AlgaeArmIntake extends SubsystemBase {
    private final SparkMax AlgaeArmIntakeMotor = new SparkMax(AlgaeArmIntakeConstants.kAlgaeArmIntakeMotorCanID,
            MotorType.kBrushless);
    private final DigitalInput AlgaeArmIntakeSensor = new DigitalInput(
            AlgaeArmIntakeConstants.kAlgaeArmIntakeSensorPort);

    public AlgaeArmIntake() {
        SparkMaxConfig AlgaeArmIntakeMotorConfig = new SparkMaxConfig();
        AlgaeArmIntakeMotorConfig.inverted(AlgaeArmIntakeConstants.kAlgaeArmIntakeMotorInverted);
        AlgaeArmIntakeMotor.configure(AlgaeArmIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command grabCommand() {
        return new Command() {
            @Override
            public void initialize() {
                setIntakeSpeed(1);
            }

            @Override
            public void end(boolean interrupted) {
                setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished() {
                return getAlgaeArmIntakeSensor();
            }
        };
    }

    public Command releaseCommand() {
        return new Command() {
            @Override
            public void initialize() {
                setIntakeSpeed(-1);
            }

            @Override
            public void end(boolean interrupted) {
                setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished() {
                return !getAlgaeArmIntakeSensor();
            }
        };
    }

    public Command stopMotors() {
        return new InstantCommand(() -> setIntakeSpeed(0));
    }

    public void setIntakeSpeed(double speed) {
        AlgaeArmIntakeMotor.set(speed);
    }

    public boolean getAlgaeArmIntakeSensor() {
        return AlgaeArmIntakeSensor.get();
    }
}
