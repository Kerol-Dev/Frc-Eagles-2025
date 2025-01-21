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
import frc.robot.Constants.AlgaeArmIntakeConstants;
import frc.robot.Constants.CoralArmIntakeConstants;

public class Intake extends SubsystemBase{
    private final SparkMax coralArmIntakeMotor = new SparkMax(CoralArmIntakeConstants.kCoralArmIntakeMotorCanID, MotorType.kBrushless);
    private final DigitalInput coralArmIntakeSensor = new DigitalInput(CoralArmIntakeConstants.kCoralArmIntakeSensorPort);
    private final DigitalInput algaeArmIntakeSensor = new DigitalInput(AlgaeArmIntakeConstants.kAlgaeArmIntakeSensorPort);

    public Intake(){
        SparkMaxConfig coralArmIntakeMotorConfig = new SparkMaxConfig();
        coralArmIntakeMotorConfig.inverted(CoralArmIntakeConstants.kCoralArmIntakeMotorInverted);
        coralArmIntakeMotor.configure(coralArmIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Sensor", getCoralIntakeSensor());
        SmartDashboard.putNumber("Coral Temp", coralArmIntakeMotor.getMotorTemperature());
    }

    public Command grabCommand(boolean isAlgae){
        return new Command(){
            @Override
            public void initialize(){
                setIntakeSpeed(1 * (isAlgae ? -1 : 1));
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

    public Command releaseCommand(boolean isAlgae){
        return new Command(){
            @Override
            public void initialize(){
                setIntakeSpeed(-0.25 * (isAlgae ? -1 : 1));
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

    public boolean getAlgaeArmIntakeSensor()
    {
        return !algaeArmIntakeSensor.get();
    }
}
