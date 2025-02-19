package frc.robot;

import java.util.function.BooleanSupplier;

// Import necessary libraries and classes
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmRotationIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.misc.ArmPosition;
import frc.robot.subsystems.misc.ElevatorPosition;
import frc.robot.subsystems.misc.PressCountCommandHandler;
import frc.robot.subsystems.pathfind.PathfindType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/**
 * Main container for the robot. Handles subsystems, commands, and button bindings.
 */
public class RobotContainer {

  // PS5 controller for driver input
  public static final CommandPS5Controller driverController = new CommandPS5Controller(
      OIConstants.kDriverControllerPort);

  // Subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_Intake = new Intake();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmRotationIntake m_arm = new ArmRotationIntake();

  // Autonomous command chooser
  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Slow speed toggle flag
  public boolean slowSpeedEnabled = false;

  // Object Mode
  public static boolean coralMode = true;

  // Command handlers for multi-step actions
  public final PressCountCommandHandler placeCoralCommand;
  public final PressCountCommandHandler grabAlgaeCommand;

  /**
   * Constructor for the RobotContainer.
   * Initializes subsystems, commands, and button bindings.
   */
  public RobotContainer() {

    // ------------------------------------------------
    // Updated PressCountCommandHandlers
    // ------------------------------------------------
    // Each press runs:
    //  1) A conditional switch to coral mode/algae mode
    //  2) Pathfinding to the correct location
    //  3) Elevator/arm commands for that "step"

    placeCoralCommand = new PressCountCommandHandler(
        /* resetCommand when exceeding # of steps: */ IdleSystemsCommand(),
        /* minimum press interval: */ 0.25,

        // Step 1 -> L1
        checkAndSwitchToCoralMode()
            .andThen(pathfindToReef())
            .andThen(PlaceReefCoralCommand(ElevatorPosition.place_coral_l1, ArmPosition.place_coral_l1)),

        // Step 2 -> L2
        checkAndSwitchToCoralMode()
            .andThen(pathfindToReef())
            .andThen(PlaceReefCoralCommand(ElevatorPosition.place_coral_l2, ArmPosition.place_coral_l2)),

        // Step 3 -> L3
        checkAndSwitchToCoralMode()
            .andThen(pathfindToReef())
            .andThen(PlaceReefCoralCommand(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)),

        // Step 4 -> L4
        checkAndSwitchToCoralMode()
            .andThen(pathfindToReef())
            .andThen(PlaceReefCoralCommand(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4))
    );

    grabAlgaeCommand = new PressCountCommandHandler(
        /* resetCommand: */ IdleSystemsCommand(),
        /* min press interval: */ 0.25,

        // First press -> Algae Reef 1
        checkAndSwitchToAlgaeMode()
            .andThen(pathFindToAlgae())
            .andThen(GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_1, ArmPosition.grab_algae_reef_1)),

        // Second press -> Algae Reef 2
        checkAndSwitchToAlgaeMode()
            .andThen(pathFindToAlgae())
            .andThen(GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_2, ArmPosition.grab_algae_reef_2))
    );

    // Set default command for the drivetrain
    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.4,
            true,
            slowSpeedEnabled),
        m_robotDrive));

    // Configure button bindings
    configureButtonBindings();

    // Configure autonomous routines
    configurePathPlanner();
  }

  /**
   * Configures PathPlanner autonomous routines.
   */
  private void configurePathPlanner() {
    // Example of how named commands are registered:
    registerNamedCommand("InitializeForSource", IntakeSourceInitCommand(), () -> m_Intake.getCoralIntakeSensor(), true);
    registerNamedCommand("GrabFromSource",
        IntakeSourceGrabCommand().andThen(IdleSystemsCommand().withTimeout(0.5)),
        () -> m_Intake.getCoralIntakeSensor(),
        true);

    registerNamedCommand("PlaceL4Init", PlaceReefInit(ElevatorPosition.place_coral_l4),
        () -> m_Intake.getCoralIntakeSensor(), true);
    registerNamedCommand("PlaceL4",
        PlaceReefCoralCommand(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
            .andThen(IdleSystemsCommand().withTimeout(1.2)),
        () -> m_Intake.getCoralIntakeSensor(),
        true);

    registerNamedCommand("SpitAlgae",
        DropAlgaeProcessorCommand().andThen(IdleSystemsCommand().withTimeout(0.5)),
        () -> !m_Intake.getAlgaeArmIntakeSensor(),
        false);
    registerNamedCommand("InitAlgae1", PlaceReefInit(ElevatorPosition.grab_algae_reef_1),
        () -> !m_Intake.getAlgaeArmIntakeSensor(),
        false);
    registerNamedCommand("InitAlgae2", PlaceReefInit(ElevatorPosition.grab_algae_reef_2),
        () -> !m_Intake.getAlgaeArmIntakeSensor(),
        false);
    registerNamedCommand("GrabAlgae1",
        GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_1, ArmPosition.grab_algae_reef_1)
            .andThen(IdleSystemsCommand().withTimeout(1.2)),
        () -> !m_Intake.getAlgaeArmIntakeSensor(),
        false);
    registerNamedCommand("GrabAlgae2",
        GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_2, ArmPosition.grab_algae_reef_2)
            .andThen(IdleSystemsCommand().withTimeout(1.2)),
        () -> !m_Intake.getAlgaeArmIntakeSensor(),
        false);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  private void registerNamedCommand(String name, Command command, BooleanSupplier condition, boolean isCoralMode) {
    NamedCommands.registerCommand(
        name,
        new InstantCommand(() -> {
          if (coralMode != isCoralMode) {
            // Switch modes if mismatch
            SwitchObjectMode().schedule();
          }
        }).andThen(command.onlyIf(condition)));
  }

  /**
   * Maps controller buttons to specific commands.
   */
  private void configureButtonBindings() {
    // Example: Zero the heading if needed
    // driverController.s().onTrue(new InstantCommand(m_robotDrive::zeroHeading));

    // Toggle slow speed
    // driverController.L1().onTrue(new InstantCommand(() -> slowSpeedEnabled = !slowSpeedEnabled));

    // -------------------------------------------------------------------------
    // Revised R2: Press -> increment placeCoralCommand to next step
    // (which itself does the mode switch, pathfind, and place sequence)
    // -------------------------------------------------------------------------
    driverController.R2().and(driverController.povUp()).whileTrue(checkAndSwitchToCoralMode()
    .andThen(pathfindToReef())
    .andThen(PlaceReefCoralCommand(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)));

    // -------------------------------------------------------------------------
    // Revised R1: Press -> increment grabAlgaeCommand to next step
    // (which does the mode switch, pathfind, and grab sequence)
    // -------------------------------------------------------------------------
    driverController.R1()
      .whileTrue(checkAndSwitchToAlgaeMode().andThen(pathFindToAlgae().andThen(IntakeSourceGrabCommand())))
       .onFalse(IdleSystemsCommand());
    // -------------------------------------------------------------------------
    // (Optional) L2, L1, etc. remain unchanged if you wish:
    // Example usage (commented out if not needed):
    //
    driverController.L2()
      .whileTrue(checkAndSwitchToCoralMode().andThen(pathfindToHuman().andThen(IntakeSourceGrabCommand())))
       .onFalse(IdleSystemsCommand());
     driverController.L1()
         .whileTrue(checkAndSwitchToAlgaeMode().andThen(pathfindToProcessor().andThen(DropAlgaeProcessorCommand())))
         .onFalse(IdleSystemsCommand());
    //
    // Adjust or remove as needed.
    // -------------------------------------------------------------------------
  }

  /**
   * Command to switch object mode (toggles coral/algae). Resets elevator & arm to idle.
   */
  private Command SwitchObjectMode() {
    return new InstantCommand(() -> coralMode = !coralMode)
        .andThen(m_elevator.setElevatorPositionCommand(ElevatorPosition.idle))
        .andThen(m_arm.setArmPositionCommand(ArmPosition.idle));
  }

  /**
   * Checks if we need to switch to coral mode; if so, do it.
   */
  private Command checkAndSwitchToCoralMode() {
    return new ConditionalCommand(
        SwitchObjectMode(),
        new InstantCommand(), // Do nothing if already in coral mode
        () -> !coralMode
    );
  }

  /**
   * Checks if we need to switch to algae mode; if so, do it.
   */
  private Command checkAndSwitchToAlgaeMode() {
    return new ConditionalCommand(
        SwitchObjectMode(),
        new InstantCommand(), // Do nothing if already in algae mode
        () -> coralMode
    );
  }

  /**
   * Command for placing coral at the reef.
   */
  private Command PlaceReefCoralCommand(ElevatorPosition elevatorPosition, ArmPosition armPosition) {
    return PlaceReefInit(elevatorPosition)
    .andThen(Commands.print("TEST"))
        .andThen(m_arm.setArmPositionCommand(armPosition))
        .andThen(AutoReleaseCoral())
        .andThen(m_arm.setArmPositionCommand(ArmPosition.idle));
  }

  /**
   * Automatically releases coral and triggers rumble.
   */
  private Command AutoReleaseCoral() {
    return m_Intake.releaseCommand(true)
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Prepares elevator for reef placement.
   */
  private Command PlaceReefInit(ElevatorPosition elevatorPosition) {
    return m_arm.setArmPositionCommand(ArmPosition.idle)
        .andThen(m_elevator.setElevatorPositionCommand(elevatorPosition));
  }

  /**
   * Command for grabbing algae at the reef.
   */
  private Command GrabAlgaeReefCommand(ElevatorPosition elevatorPosition, ArmPosition armPosition) {
    return m_elevator.setElevatorPositionCommand(elevatorPosition)
        .alongWith(m_arm.setArmPositionCommand(armPosition))
        .andThen(m_Intake.grabCommand(false))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command for dropping algae at the processor.
   */
  private Command DropAlgaeProcessorCommand() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.place_algae_processor)
        .alongWith(m_arm.setArmPositionCommand(ArmPosition.place_algae_processor))
        .andThen(m_Intake.releaseCommand(false))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command for grabbing from the intake source.
   */
  private Command IntakeSourceGrabCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.grab_coral_source)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.grab_coral_source))
        .andThen(m_Intake.grabCommand(false))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  private Command IntakeSourceInitCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.grab_coral_source)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.grab_coral_source));
  }

  /**
   * Command to set all systems to idle state.
   */
  private Command IdleSystemsCommand() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.idle)
        .alongWith(m_arm.setArmPositionCommand(ArmPosition.idle))
        .alongWith(m_Intake.stopMotors());
  }

  /**
   * Triggers controller rumble for a specified duration (ignored in auto).
   */
  private void triggerRumble(double durationSeconds) {
    if (DriverStation.isAutonomousEnabled()) {
      return;
    }
    driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    new Thread(() -> {
      try {
        Thread.sleep((long) (durationSeconds * 1000));
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      }
    }).start();
  }

  /**
   * Returns the selected autonomous command.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // ----------------------------------------------------------------
  // Pathfinding Helpers
  // ----------------------------------------------------------------
  private Command pathfindToHuman() {
    return m_robotDrive.goToPosePathfind(PathfindType.Human, () -> m_robotDrive.getPose());
  }

  private Command pathfindToReef() {
    return m_robotDrive.goToPosePathfind(PathfindType.Reef, () -> m_robotDrive.getPose());
  }

  private Command pathFindToAlgae() {
    return m_robotDrive.goToPosePathfind(PathfindType.Algea, () -> m_robotDrive.getPose());
  }

  private Command pathfindToProcessor() {
    return m_robotDrive.goToPosePathfind(PathfindType.Processor, () -> m_robotDrive.getPose());
  }
}