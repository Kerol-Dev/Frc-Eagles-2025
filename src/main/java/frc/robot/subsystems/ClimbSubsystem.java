package frc.robot.subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private static final SparkMax climbMotor = new SparkMax(ClimbConstants.kClimbMotorCanID, MotorType.kBrushless);
    SparkClosedLoopController climbMotorController = climbMotor.getClosedLoopController();
    public static Servo servo = new Servo(0);

    public static void setClimbConfiguration(boolean fmsConnected) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.softLimit.forwardSoftLimitEnabled(fmsConnected);
        config.softLimit.forwardSoftLimit(ClimbConstants.kClimbMotorForwardSoftLimit);
        config.softLimit.reverseSoftLimitEnabled(fmsConnected);
        config.softLimit.reverseSoftLimit(ClimbConstants.kClimbMotorReverseSoftLimit);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public ClimbSubsystem() {
        setClimbConfiguration(false);
        climbMotor.getEncoder().setPosition(0);
    }

    public void setClimbSpeed(double speed) {
        climbMotor.set(speed);
    }

    public void stop() {
        climbMotor.stopMotor();
    }           
    
    public void openServo(boolean climb)
    {
        servo.setAngle(!climb ? 80 : 150);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Climb Position", climbMotor.getEncoder().getPosition());
    }
}