package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralArmIntakeConstants;

public class CoralIntake extends SubsystemBase{
    private final SparkMax coralArmIntakeMotor = new SparkMax(CoralArmIntakeConstants.kCoralArmIntakeMotorCanID, MotorType.kBrushless);
    private final DigitalInput coralArmIntakeSensor = new DigitalInput(CoralArmIntakeConstants.kCoralArmIntakeSensorPort);

    public CoralIntake(){
        SparkMaxConfig coralArmIntakeMotorConfig = new SparkMaxConfig();
        coralArmIntakeMotorConfig.inverted(CoralArmIntakeConstants.kCoralArmIntakeMotorInverted);
        coralArmIntakeMotor.configure(coralArmIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Sensor", getCoralIntakeSensor());
        SmartDashboard.putNumber("Coral Temp", coralArmIntakeMotor.getMotorTemperature());
    }

    public Command grabCommand(){
        return new Command(){
            @Override
            public void initialize(){
                setIntakeSpeed(1);
            }

            @Override
            public void end(boolean interrupted){
                setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished(){
                return getCoralIntakeSensor();
            }
        };
    }

    public Command releaseCommand(){
        return new Command(){
            @Override
            public void initialize(){
                setIntakeSpeed(-0.25);
            }

            @Override
            public void end(boolean interrupted){
                setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished(){
                return !getCoralIntakeSensor();
            }
        };
    }

    public Command stopMotors(){
        return new InstantCommand(() -> setIntakeSpeed(0));
    }

    public void setIntakeSpeed(double speed){
        coralArmIntakeMotor.set(speed);
    }

    public boolean getCoralIntakeSensor(){
        return !coralArmIntakeSensor.get();
    }
}
