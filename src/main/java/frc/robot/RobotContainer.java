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
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.misc.ArmPosition;
import frc.robot.subsystems.misc.ElevatorPosition;
import frc.robot.subsystems.pathfind.PathfindType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Main container for the robot. Handles subsystems, commands, and button
 * bindings.
 */
public class RobotContainer {

  // Xbox controller for driver input
  public static final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final PivotSubsystem m_arm = new PivotSubsystem();
  // private final LedSubsystem m_LedSubsystem = new LedSubsystem();

  // Autonomous command chooser
  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Slow speed toggle flag
  public boolean slowSpeedEnabled = false;

  // Object Mode
  public static boolean coralMode = true;

  /**
   * Constructor for the RobotContainer.
   * Initializes subsystems, commands, and button bindings.
   */
  public RobotContainer() {
    // Set default command for the drivetrain
    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.2,
            true,
            slowSpeedEnabled),
        m_robotDrive));

    // m_LedSubsystem.setDefaultCommand(new RunCommand(() ->
    // m_LedSubsystem.updateFireAnimation(), m_LedSubsystem));

    // Configure button bindings
    configureButtonBindings();

    // Configure autonomous routines
    configurePathPlanner();
  }

  /**
   * Configures PathPlanner autonomous routines.
   */
  private void configurePathPlanner() {
    registerNamedCommand("GrabFromSource", IntakeSourceGrabCommand().andThen(IdleSystemsCommand()),
        () -> !m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("PlaceL4", PlaceReefCoralCommand(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
        .andThen(IdleSystemsCommand()), () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("PlaceL3", PlaceReefCoralCommand(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
        .andThen(IdleSystemsCommand()), () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("SpitAlgae", DropAlgaeProcessorCommand().andThen(IdleSystemsCommand()),
        () -> !m_Intake.getAlgaeArmIntakeSensor(), false);

    registerNamedCommand("GrabAlgae1",
        GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_1, ArmPosition.grab_algae_reef_1)
            .andThen(IdleSystemsCommand()),
        () -> !m_Intake.getAlgaeArmIntakeSensor(), false);

    registerNamedCommand("GrabAlgae2",
        GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_2, ArmPosition.grab_algae_reef_2)
            .andThen(IdleSystemsCommand()),
        () -> !m_Intake.getAlgaeArmIntakeSensor(), false);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  private void registerNamedCommand(String name, Command command, BooleanSupplier condition, boolean isCoralMode) {
    NamedCommands.registerCommand(name, new ConditionalCommand(
        SwitchObjectMode(),
        new InstantCommand(),
        () -> coralMode != isCoralMode).andThen(command).onlyIf(condition));
  }

  /**
   * Maps controller buttons to specific commands.
   */
  private void configureButtonBindings() {
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    driverController.b().onTrue(checkAndSwitchToCoralMode().andThen(
        pathfindToHuman().andThen(m_Intake.grabCommand(false)).onlyIf(() -> !m_Intake.getCoralIntakeSensor())));

    // driverController.a().whileTrue(m_Intake.grabCommand(false).onlyIf(() -> !m_Intake.getCoralIntakeSensor()))
    // .onFalse(m_Intake.stopMotors());\

    driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.configureAutoBuilder()));

    driverController.rightBumper().onTrue(
        checkAndSwitchToCoralMode().andThen(pathfindToReef(true)).onlyIf(() -> m_Intake.getCoralIntakeSensor()));

    driverController.leftBumper().onTrue(
        checkAndSwitchToCoralMode().andThen(pathfindToReef(false)).onlyIf(() -> m_Intake.getCoralIntakeSensor()));

    // TODO: To remove
    driverController.x().whileTrue(m_Intake.releaseCommand(true));

    driverController.povUp().onTrue(PlaceReefCoralCommand(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
        .andThen(IdleSystemsCommand()).onlyIf(() -> m_Intake.getCoralIntakeSensor()));
    driverController.povDown().onTrue(PlaceReefCoralCommand(ElevatorPosition.place_coral_l2, ArmPosition.place_coral_l4)
        .andThen(IdleSystemsCommand()).onlyIf(() -> m_Intake.getCoralIntakeSensor()));
    driverController.povRight()
        .onTrue(PlaceReefCoralCommand(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l4)
            .andThen(IdleSystemsCommand()).onlyIf(() -> m_Intake.getCoralIntakeSensor()));
    driverController.povLeft().onTrue(
        pathfindToProcessor().andThen(DropAlgaeProcessorCommand()).onlyIf(() -> m_Intake.getAlgaeArmIntakeSensor()));

    driverController.leftTrigger()
        .whileTrue(checkAndSwitchToAlgaeMode().andThen(pathFindToAlgae()
            .andThen(GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_1, ArmPosition.grab_algae_reef_1)
                .andThen(m_Intake.grabCommand(true)))))
        .onFalse(IdleSystemsCommand());

    driverController.rightTrigger()
        .whileTrue(checkAndSwitchToAlgaeMode().andThen(pathFindToAlgae()
            .andThen(GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_2, ArmPosition.grab_algae_reef_2)
                .andThen(m_Intake.grabCommand(true)))))
        .onFalse(IdleSystemsCommand());

    driverController.y()
        .onTrue(IdleSystemsCommand().andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())));
  }

  private Command checkAndSwitchToCoralMode() {
    return new ConditionalCommand(
        SwitchObjectMode(),
        new InstantCommand(),
        () -> !coralMode);
  }

  private Command checkAndSwitchToAlgaeMode() {
    return new ConditionalCommand(
        SwitchObjectMode(),
        new InstantCommand(),
        () -> coralMode);
  }

  /**
   * Command for placing coral at the reef.
   * 
   * @param elevatorPosition Elevator position
   * @param armPosition      Arm position
   * @return Place reef coral command
   */
  private Command PlaceReefCoralCommand(ElevatorPosition elevatorPosition, ArmPosition armPosition) {
    return PlaceReefInit(elevatorPosition)
        .andThen(m_arm.setArmPositionCommand(armPosition))
        .andThen(new WaitCommand(0.4))
        .andThen(AutoReleaseCoral());
  }

  private Command AutoReleaseCoral() {
    return m_Intake.releaseCommand(false)
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  private Command SwitchObjectMode() {
    return new InstantCommand(() -> {
      coralMode = !coralMode;
      if (coralMode) {
        m_elevator.setElevatorPosition(0.4);
      } else {
        m_elevator.setElevatorPosition(1.54);
      }
    }).andThen(new WaitUntilCommand(() -> m_elevator.isElevatorAtPosition())).andThen(IdleSystemsCommand());
  }

  private Command PlaceReefInit(ElevatorPosition elevatorPosition) {
    return m_arm.setArmPositionCommand(ArmPosition.idle)
        .andThen(m_elevator.setElevatorPositionCommand(elevatorPosition));
  }

  /**
   * Command for grabbing algae at the reef.
   * 
   * @param position Elevator position
   * @return Grab algae reef command
   */
  private Command GrabAlgaeReefCommand(ElevatorPosition elevatorPosition, ArmPosition armPosition) {
    return m_elevator.setElevatorPositionCommand(elevatorPosition)
        .alongWith(m_arm.setArmPositionCommand(armPosition))
        .andThen(m_Intake.grabCommand(true))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command for dropping algae at the processor.
   * 
   * @return Drop algae processor command
   */
  private Command DropAlgaeProcessorCommand() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.place_algae_processor)
        .alongWith(m_arm.setArmPositionCommand(ArmPosition.place_algae_processor))
        .andThen(m_Intake.releaseCommand(true))
        .andThen(new WaitUntilCommand(() -> !m_Intake.getAlgaeArmIntakeSensor()))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command for grabbing from the intake source.
   * 
   * @return Intake source grab command
   */
  private Command IntakeSourceGrabCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.grab_coral_source)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.grab_coral_source))
        .andThen(m_Intake.grabCommand(false))
        .andThen(new InstantCommand(() -> m_Intake.setIntakeSpeed(0.25)).withTimeout(0.3))
        .andThen(new InstantCommand(() -> m_Intake.setIntakeSpeed(0)))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command to set all systems to idle state.
   * 
   * @return Idle systems command
   */
  private Command IdleSystemsCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.idle)
        .andThen(m_elevator.setElevatorPositionCommand(ElevatorPosition.idle));
  }

  /**
   * Triggers controller rumble for a specified duration.
   * 
   * @param durationSeconds Duration in seconds
   */
  private void triggerRumble(double durationSeconds) {
    if (DriverStation.isAutonomousEnabled())
      return;

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
   * 
   * @return Selected autonomous command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private Command resetCommandScheduler() {
    return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
  }

  // Pathfinding Commands
  private Command pathfindToHuman() {
    return m_robotDrive
        .goToPosePathfind(PathfindType.Human, false)
        .andThen(new WaitUntilCommand(() -> m_robotDrive.finishedPath()))
        .andThen(resetCommandScheduler());
  }

  private Command pathfindToReef(boolean right) {
    return m_robotDrive
        .goToPosePathfind(PathfindType.Reef, right)
        .andThen(new WaitUntilCommand(() -> m_robotDrive.finishedPath()))
        .andThen(resetCommandScheduler());
  }

  private Command pathFindToAlgae() {
    return m_robotDrive.goToPosePathfind(PathfindType.Algea, false)
        .andThen(resetCommandScheduler());
  }

  private Command pathfindToProcessor() {
    return m_robotDrive.goToPosePathfind(PathfindType.Processor, false)
        .andThen(resetCommandScheduler());
  }
}