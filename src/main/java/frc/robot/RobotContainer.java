package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  final CommandXboxController driverXbox = new CommandXboxController(0);

  public RobotContainer() {
    Command teleopCommand = drivebase.teleopDrive(
        () -> driverXbox.getLeftX(),
        () -> driverXbox.getLeftY(),
        () -> -driverXbox.getRightX(),
        () -> 1);
    drivebase.setDefaultCommand(teleopCommand);
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("ex");
  }
}
