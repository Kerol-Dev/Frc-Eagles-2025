package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private boolean isREPLAY = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    if (isREPLAY) {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
    else
    {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    }

    org.littletonrobotics.junction.Logger.start();
    DriveSubsystem.resetToAbsolute();
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // m_robotContainer.m_LedSubsystem.updateShootingRayEffect();
  }

  @Override
  public void autonomousInit() {
    DriveSubsystem.resetEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null && (VisionSubsystem.getLimelightObjectTarget() || RobotContainer.autoChooser.getSelected().getName().startsWith("M"))) {
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
    m_robotContainer.IdleSystemsCommand().schedule();
  }

  boolean slowSpeedEnabledAutomatically = false;

  @Override
  public void teleopPeriodic() {
    m_robotContainer.periodic();
    if (ElevatorSubsystem.elevatorMotor.getPosition().getValueAsDouble() > 2.5) {
      slowSpeedEnabledAutomatically = true;
      m_robotContainer.slowSpeedEnabled = true;
    } else if (slowSpeedEnabledAutomatically) {
      slowSpeedEnabledAutomatically = false;
      m_robotContainer.slowSpeedEnabled = false;
    }

    Logger.recordOutput("Joystick/Active Button", m_robotContainer.getPressedJoystickButtonName());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}