package frc.robot;

import java.util.function.BooleanSupplier;

// Import necessary libraries and classes
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
    // // Set default command for the drivetrain
    // m_robotDrive.setDefaultCommand(new RunCommand(
    //     () -> m_robotDrive.drive(
    //         -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
    //         -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
    //         MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.4,
    //         true,
    //         slowSpeedEnabled),
    //     m_robotDrive));

    // m_LedSubsystem.setDefaultCommand(new RunCommand(() -> m_LedSubsystem.updateFireAnimation(), m_LedSubsystem));

    // Configure button bindings
    configureButtonBindings();

    // Configure autonomous routines
    configurePathPlanner();
  }

  /**
   * Configures PathPlanner autonomous routines.
   */
  private void configurePathPlanner() {
    registerNamedCommand("InitializeForSource", IntakeSourceInitCommand(), () -> !m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("GrabFromSource", IntakeSourceGrabCommand().andThen(IdleSystemsCommand()),
        () -> !m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("PlaceL4Init", PlaceReefInit(ElevatorPosition.place_coral_l4),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("PlaceL4", PlaceReefCoralCommand(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
        .andThen(IdleSystemsCommand()), () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("PlaceL3", PlaceReefCoralCommand(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
        .andThen(IdleSystemsCommand()), () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("SpitAlgae", DropAlgaeProcessorCommand().andThen(IdleSystemsCommand()),
        () -> !m_Intake.getAlgaeArmIntakeSensor(), false);

    registerNamedCommand("InitAlgae1", PlaceReefInit(ElevatorPosition.grab_algae_reef_1),
        () -> !m_Intake.getAlgaeArmIntakeSensor(), false);

    registerNamedCommand("InitAlgae2", PlaceReefInit(ElevatorPosition.grab_algae_reef_2),
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
      () -> coralMode != isCoralMode
    ).andThen(command).onlyIf(condition));
  }

  /**
   * Maps controller buttons to specific commands.
   */
  private void configureButtonBindings() {
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    driverController.a().onTrue(new InstantCommand(() -> slowSpeedEnabled = !slowSpeedEnabled));

    driverController.b().whileTrue(Commands.sequence(AutoBuilder.buildAuto("Auto-Algae"), AutoBuilder.buildAuto("Auto-L4-Mirrored"), AutoBuilder.buildAuto("Auto-L3-Mirrored")))
    .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    driverController.rightTrigger().and(driverController.povDown())
      .whileTrue(Commands.sequence(checkAndSwitchToCoralMode(), pathfindToReef(), PlaceReefCoralCommand(ElevatorPosition.place_coral_l2, ArmPosition.place_coral_l2), IdleSystemsCommand()))
      .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(pathfindToReef())).andThen(IdleSystemsCommand()));

    driverController.rightTrigger().and(driverController.povRight())
      .whileTrue(Commands.sequence(checkAndSwitchToCoralMode(), pathfindToReef(), PlaceReefCoralCommand(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3), IdleSystemsCommand()))
      .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(pathfindToReef())).andThen(IdleSystemsCommand()));

    driverController.rightTrigger().and(driverController.povUp())
      .whileTrue(Commands.sequence(checkAndSwitchToCoralMode(), pathfindToReef(), PlaceReefCoralCommand(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4), IdleSystemsCommand()))
      .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(pathfindToReef())).andThen(IdleSystemsCommand()));

    driverController.rightBumper().and(driverController.povDown())
      .whileTrue(Commands.sequence(checkAndSwitchToAlgaeMode(), pathFindToAlgae(), GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_1, ArmPosition.grab_algae_reef_1), IdleSystemsCommand()))
      .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(pathFindToAlgae())).andThen(IdleSystemsCommand()));

    driverController.rightBumper().and(driverController.povRight())
      .whileTrue(Commands.sequence(checkAndSwitchToAlgaeMode(), pathFindToAlgae(), GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_2, ArmPosition.grab_algae_reef_2), IdleSystemsCommand()))
      .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(pathFindToAlgae())).andThen(IdleSystemsCommand()));


    driverController.leftTrigger()
      .whileTrue(checkAndSwitchToCoralMode().andThen(pathfindToHuman().andThen(IntakeSourceGrabCommand())))
      .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(pathfindToHuman())).andThen(IdleSystemsCommand()));

    driverController.leftBumper()
      .whileTrue(checkAndSwitchToAlgaeMode().andThen(pathfindToProcessor().andThen(DropAlgaeProcessorCommand())))
      .onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(pathfindToProcessor())).andThen(IdleSystemsCommand()));
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
        .andThen(AutoReleaseCoral());
  }

  private Command AutoReleaseCoral() {
    return m_Intake.releaseCommand(false)
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  private Command SwitchObjectMode() {
    return new InstantCommand(() -> coralMode = !coralMode)
        .andThen(m_elevator.setElevatorPositionCommand(ElevatorPosition.idle))
        .andThen(m_arm.setArmPositionCommand(ArmPosition.idle));
  }

  //#region OPTIONAL
  // private Command PlaceAutomaticCoral() {
  //   return new Command() {
  //     int currentLevel = 4;
  //     Command currentCommand = PlaceReefInit(ElevatorPosition.place_coral_l4);
  //     boolean isFinished = false;

  //     @Override
  //     public void initialize() {
  //       currentCommand.schedule();
  //     }

  //     @Override
  //     public void execute() {
  //       if (currentCommand.isFinished()) {
  //         if (VisionSubsystem.getLimelightObjectTarget()) {
  //           currentLevel--;
  //         } else {
  //           isFinished = true;
  //           return;
  //         }

  //         switch (currentLevel) {
  //           case 3:
  //             currentCommand = PlaceReefInit(ElevatorPosition.place_coral_l3);
  //             break;
  //           case 2:
  //             currentCommand = PlaceReefInit(ElevatorPosition.place_coral_l2);
  //             break;
  //           case 1:
  //             isFinished = true;
  //             return;
  //         }
  //         currentCommand.schedule();
  //       }
  //     }

  //     @Override
  //     public boolean isFinished() {
  //       return isFinished;
  //     }
  //   };
  // }
  //#endregion

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

  private Command IntakeSourceInitCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.grab_coral_source)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.grab_coral_source));
  }

  /**
   * Command to set all systems to idle state.
   * 
   * @return Idle systems command
   */
  private Command IdleSystemsCommand() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.idle)
        .alongWith(m_arm.setArmPositionCommand(ArmPosition.idle))
        .alongWith(m_Intake.stopMotors());
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

  // Pathfinding Commands
  private Command pathfindToHuman() {
    return m_robotDrive.goToPosePathfind(PathfindType.Human);
  }

  private Command pathfindToReef() {
    return m_robotDrive.goToPosePathfind(PathfindType.Reef);
  }

  private Command pathFindToAlgae() {
    return m_robotDrive.goToPosePathfind(PathfindType.Algea);
  }

  private Command pathfindToProcessor() {
    return m_robotDrive.goToPosePathfind(PathfindType.Processor);
  }
}