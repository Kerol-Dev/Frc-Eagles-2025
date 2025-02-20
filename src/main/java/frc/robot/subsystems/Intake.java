package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntakeConstants;

public class Intake extends SubsystemBase{
    private final TalonFX ArmIntakeMotor = new TalonFX(ArmIntakeConstants.kArmIntakeMotorCanID);
    private final DigitalInput coralArmIntakeSensor = new DigitalInput(ArmIntakeConstants.kCoralArmIntakeSensorPort);
    private final DigitalInput algaeArmIntakeSensor = new DigitalInput(ArmIntakeConstants.kAlgaeArmIntakeSensorPort);

    public Intake(){
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = ArmIntakeConstants.kArmIntakeMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        ArmIntakeMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Sensor", getCoralIntakeSensor());
        SmartDashboard.putNumber("Coral Temp", ArmIntakeMotor.getDeviceTemp().getValueAsDouble());
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
                return isAlgae ? getAlgaeArmIntakeSensor() : getCoralIntakeSensor();
            }
        };
    }

    public Command releaseCommand(boolean isAlgae){
        return new Command(){
            @Override
            public void initialize(){
                setIntakeSpeed(-0.5 * (isAlgae ? -1 : 1));
            }

            @Override
            public void end(boolean interrupted){
                setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished(){
                return isAlgae ? !getAlgaeArmIntakeSensor() : !getCoralIntakeSensor();
            }
        };
    }

    public Command stopMotors(){
        return new InstantCommand(() -> setIntakeSpeed(0));
    }

    public void setIntakeSpeed(double speed){
        ArmIntakeMotor.set(speed);
    }

    public boolean getCoralIntakeSensor(){
        return !coralArmIntakeSensor.get();
    }

    public boolean getAlgaeArmIntakeSensor()
    {
        return !algaeArmIntakeSensor.get();
    }
}
