package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends LoggedRobot{
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    org.littletonrobotics.junction.Logger.start();
    DriveSubsystem.resetToAbsolute();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    DriveSubsystem.resetEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  boolean slowSpeedEnabledAutomatically = false;

  @Override
  public void teleopPeriodic() {
    if(m_robotContainer.m_elevator.elevatorMotor.getPosition().getValueAsDouble() > 0.25)
    {
      slowSpeedEnabledAutomatically = true;
      m_robotContainer.slowSpeedEnabled = true;
    }
    else if(slowSpeedEnabledAutomatically)
    {
      slowSpeedEnabledAutomatically = false;
      m_robotContainer.slowSpeedEnabled = false;
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}