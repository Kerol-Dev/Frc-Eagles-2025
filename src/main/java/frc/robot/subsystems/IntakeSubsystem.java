package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmIntakeConstants;
import frc.robot.subsystems.misc.ElevatorPosition;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX ArmIntakeMotor = new TalonFX(ArmIntakeConstants.kArmIntakeMotorCanID);
    public static final DigitalInput coralArmIntakeSensor = new DigitalInput(ArmIntakeConstants.kCoralArmIntakeSensorPort);
    public static final DigitalInput algaeArmIntakeSensor = new DigitalInput(ArmIntakeConstants.kAlgaeArmIntakeSensorPort);
    private boolean grab = true;
    
    public IntakeSubsystem(){
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = ArmIntakeConstants.kArmIntakeMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        ArmIntakeMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake/Coral Sensor", getCoralIntakeSensor());
        SmartDashboard.putBoolean("Intake/Algea Sensor", getAlgaeArmIntakeSensor());

        if(!RobotContainer.coralMode && grab)
        {
            setIntakeSpeed(-0.85);
        }

        SmartDashboard.putBoolean("Grab", grab);
    }

    public Command grabCommand(boolean isAlgae){
        Command command = new Command(){
            @Override
            public void initialize(){
                grab = true;
                setIntakeSpeed(isAlgae ? -1 : 0.2);
            }

            @Override
            public void end(boolean interrupted){
                RobotContainer.triggerRumble(0.5);
                setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished(){
                return isAlgae ? getAlgaeArmIntakeSensor() : getCoralIntakeSensor();
            }
        };
        command.addRequirements(this);
        return command;
    }

    public Command releaseCommand(boolean isAlgae, ElevatorPosition position){
        return new Command(){
            @Override
            public void initialize(){
                grab = false;
                if(position == ElevatorPosition.place_coral_l1)
                {
                    setIntakeSpeed(0.2);
                }
                else if(isAlgae)
                {
                    if(position == ElevatorPosition.place_algae_processor)
                    {
                        setIntakeSpeed(0.2);
                    }
                    else
                    {
                        setIntakeSpeed(1);
                    }
                }
                else
                {
                    setIntakeSpeed(0.55);
                }
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
