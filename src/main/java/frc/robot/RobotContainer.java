package frc.robot;

// Import necessary libraries and classes
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Main container for the robot. Handles subsystems, commands, and button
 * bindings.
 */
public class RobotContainer {

  // Xbox controller for driver input
  public static final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  // Drive subsystem for controlling the robot's drivetrain
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Autonomous command chooser for selecting autonomous routines
  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Flag to toggle slow speed mode
  private boolean slowSpeedEnabled = false;

  /**
   * Constructor for the RobotContainer.
   * Initializes subsystems, sets default commands, and configures button
   * bindings.
   */
  public RobotContainer() {
    // Set the default command for the drivetrain to drive using the Xbox controller
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), // Forward/backward
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband), // Strafe
                MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.4, // Rotation
                true, // Field-relative driving
                slowSpeedEnabled // Enable/disable slow speed mode
            ),
            m_robotDrive));

    // Configure button bindings for controller input
    configureButtonBindings();

    // Configure autonomous routines
    configurePathPlanner();
  }

  /**
   * Configures autonomous routines using PathPlanner.
   */
  private void configurePathPlanner() {
    // Initialize the autonomous command chooser using PathPlanner's AutoBuilder
    autoChooser = AutoBuilder.buildAutoChooser();
  }

  /**
   * Maps controller buttons to specific commands.
   */
  private void configureButtonBindings() {
    // Bind "Start" button to reset the robot's heading
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // Bind "Left Bumper" button to toggle slow speed mode
    driverController.leftBumper().onTrue(new InstantCommand(() -> slowSpeedEnabled = !slowSpeedEnabled));
  }

  /**
   * Returns the selected autonomous command.
   * 
   * @return The selected autonomous command.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}