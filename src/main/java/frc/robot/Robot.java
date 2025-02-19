package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot{
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
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

    Commands.sequence(m_robotContainer.m_robotDrive.goToPose("reef_b"), new WaitCommand(1), 
    m_robotContainer.m_robotDrive.goToPose("human_left"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("reef_c"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("human_right"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("reef_d"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("human_right"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("algea_b"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("processor"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("reef_e"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("human_right"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("reef_f"), new WaitCommand(1),
    m_robotContainer.m_robotDrive.goToPose("human_right"), new WaitCommand(1)
    ).schedule();
  }

  boolean slowSpeedEnabledAutomatically = false;

  @Override
  public void teleopPeriodic() {
    m_robotContainer.placeCoralCommand.executeIfDebounced();
    m_robotContainer.grabAlgaeCommand.executeIfDebounced();
    
    if(m_robotContainer.m_elevator.elevatorMotor.getPosition().getValueAsDouble() > 100)
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