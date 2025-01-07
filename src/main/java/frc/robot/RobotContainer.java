package frc.robot;

// Import necessary libraries and classes
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeArmIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralArmIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.misc.ArmPosition;
import frc.robot.subsystems.misc.ElevatorPosition;
import frc.robot.subsystems.pathfind.PathfindType;
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
  public final GroundIntakeSubsystem m_groundIntake = new GroundIntakeSubsystem();
  public final CoralArmIntake m_coralArmIntake = new CoralArmIntake();
  public final AlgaeArmIntake m_algaeArmIntake = new AlgaeArmIntake();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();

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

    driverController.leftTrigger().whileTrue(grabGroundCoral().andThen(idleAllSubsystems())).onFalse(cleanUpInterrupted().andThen(idleAllSubsystems()));
  }

  private Command grabGroundCoral() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.idle)
        .alongWith(m_arm.setArmPositionCommand(ArmPosition.grab))
        .andThen(m_groundIntake.setIntakePositionCommand(true)).andThen(m_groundIntake.grabCoral())
        .andThen(new InstantCommand(() -> triggerRumble(0.5)))
        .andThen(m_groundIntake.setIntakePositionCommand(false))
        .andThen(m_coralArmIntake.grabCommand().alongWith(m_groundIntake.spitCoral()))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  private void triggerRumble(double durationSeconds) {
    driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0); // Start rumble
    new Thread(() -> {
      try {
        Thread.sleep((long) (durationSeconds * 1000)); // Convert seconds to milliseconds
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0); // Stop rumble
      }
    }).start();
  }

  private Command cleanUpInterrupted() {
    return new InstantCommand(() -> {
      m_groundIntake.setIntakePosition(false);
      m_groundIntake.stopMotors();
      m_coralArmIntake.stopMotors();
    });
  }

  private Command placeCoral(ElevatorPosition ePosition, ArmPosition aPosition) {
    return m_elevator.setElevatorPositionCommand(ePosition).andThen(m_arm.setArmPositionCommand(aPosition))
        .andThen(m_elevator.lowerElevatorToPlace())
        .andThen(m_coralArmIntake.releaseCommand())
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  private Command pathfindToClosest() {
    return m_robotDrive.goToPosePathfind(PathfindType.Closest);
  }

  // private Command pathfindToReef()
  // {
  // return m_robotDrive.goToPosePathfind(PathfindType.Reef);
  // }

  // private Command pathfindToHuman()
  // {
  // return m_robotDrive.goToPosePathfind(PathfindType.Human);
  // }

  // private Command pathfindToProcessor()
  // {
  // return m_robotDrive.goToPosePathfind(PathfindType.Processor);
  // }

  private Command idleAllSubsystems() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.idle)
        .alongWith(m_arm
            .setArmPositionCommand(m_coralArmIntake.getCoralArmIntakeSensor() ? ArmPosition.idle : ArmPosition.grab));
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