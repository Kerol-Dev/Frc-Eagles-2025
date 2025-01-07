package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralArmIntakeConstants;

public class CoralArmIntake extends SubsystemBase{
    private final SparkMax coralArmIntakeMotor = new SparkMax(CoralArmIntakeConstants.kCoralArmIntakeMotorCanID, MotorType.kBrushless);
    private final DigitalInput coralArmIntakeSensor = new DigitalInput(CoralArmIntakeConstants.kCoralArmIntakeSensorPort);

    public CoralArmIntake(){
        SparkMaxConfig coralArmIntakeMotorConfig = new SparkMaxConfig();
        coralArmIntakeMotorConfig.inverted(CoralArmIntakeConstants.kCoralArmIntakeMotorInverted);
        coralArmIntakeMotor.configure(coralArmIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
                return getCoralArmIntakeSensor();
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
                return !getCoralArmIntakeSensor();
            }
        };
    }

    public void stopMotors(){
        setIntakeSpeed(0);
    }

    public void setIntakeSpeed(double speed){
        coralArmIntakeMotor.set(speed);
    }

    public boolean getCoralArmIntakeSensor(){
        return coralArmIntakeSensor.get();
    }
}
