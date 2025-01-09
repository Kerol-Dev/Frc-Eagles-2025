package frc.robot;

// Import necessary libraries and classes
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ArmRotationIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.misc.ArmPosition;
import frc.robot.subsystems.misc.ElevatorPosition;
import frc.robot.subsystems.misc.PressCountCommandHandler;
import frc.robot.subsystems.pathfind.PathfindType;
import frc.robot.subsystems.vision.VisionSubsystem;
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

  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralIntake m_coralArmIntake = new CoralIntake();
  private final AlgaeIntake m_algaeArmIntake = new AlgaeIntake();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmRotationIntake m_arm = new ArmRotationIntake();
  private final GroundIntakeSubsystem m_groundIntake = new GroundIntakeSubsystem();

  // Autonomous command chooser
  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Slow speed toggle flag
  public boolean slowSpeedEnabled = false;

  // Command handlers for multi-step actions
  public final PressCountCommandHandler placeCoralCommand;
  public final PressCountCommandHandler grabAlgaeCommand;

  /**
   * Constructor for the RobotContainer.
   * Initializes subsystems, commands, and button bindings.
   */
  public RobotContainer() {
    // Initialize command handlers
    placeCoralCommand = new PressCountCommandHandler(IdleSystemsCommand(), 0.25,
        PlaceReefCoralCommand(ElevatorPosition.coral_L1, ArmPosition.coral_L1),
        PlaceReefCoralCommand(ElevatorPosition.coral_L2, ArmPosition.coral_L23),
        PlaceReefCoralCommand(ElevatorPosition.coral_L3, ArmPosition.coral_L23),
        PlaceReefCoralCommand(ElevatorPosition.coral_L4, ArmPosition.coral_L4));

    grabAlgaeCommand = new PressCountCommandHandler(IdleSystemsCommand(), 0.25,
        GrabAlgaeReefCommand(ElevatorPosition.grab_algea_1),
        GrabAlgaeReefCommand(ElevatorPosition.grab_algea_2));

    // Set default command for the drivetrain
    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.4,
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
    NamedCommands.registerCommand("InitializeForSource",
        IntakeSourceInitCommand().onlyIf(() -> m_coralArmIntake.getCoralIntakeSensor()));
    NamedCommands.registerCommand("GrabFromSource", IntakeSourceGrabCommand()
        .onlyIf(() -> m_coralArmIntake.getCoralIntakeSensor()).andThen(IdleSystemsCommand().withTimeout(0.4)));
    NamedCommands.registerCommand("PlaceL4Init",
        PlaceReefCoralInitCommand(ElevatorPosition.coral_L4, ArmPosition.coral_L4)
            .onlyIf(() -> m_coralArmIntake.getCoralIntakeSensor()));
    NamedCommands.registerCommand("PlaceL4", PlaceReefCoralCommand(ElevatorPosition.coral_L4, ArmPosition.coral_L4)
        .onlyIf(() -> m_coralArmIntake.getCoralIntakeSensor()).andThen(IdleSystemsCommand().withTimeout(0.4)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Maps controller buttons to specific commands.
   */
  private void configureButtonBindings() {
    driverController.start().onTrue(new InstantCommand(m_robotDrive::zeroHeading));
    driverController.leftBumper().onTrue(new InstantCommand(() -> slowSpeedEnabled = !slowSpeedEnabled));

    // driverController.rightTrigger()
    //     .whileTrue(pathfindToReef().andThen(new InstantCommand(placeCoralCommand::recordPress)));
    driverController.rightTrigger()
        .whileTrue(pathfindToReef().andThen(PlaceAutomaticCoral().andThen(AutoReleaseCoral()).andThen(new InstantCommand(() -> triggerRumble(0.5)))));
    driverController.rightBumper()
        .whileTrue(pathFindToAlgae().andThen(new InstantCommand(grabAlgaeCommand::recordPress)));
    driverController.leftTrigger().whileTrue(pathfindToHuman().andThen(IntakeSourceGrabCommand()))
        .onFalse(IdleSystemsCommand());
    driverController.leftBumper().whileTrue(pathfindToProcessor().andThen(DropAlgaeProcessorCommand()))
        .onFalse(IdleSystemsCommand());
    driverController.a().whileTrue(GrabFromGroundCommand()).onFalse(IdleSystemsCommand());
  }

  /**
   * Command to grab from the ground.
   * 
   * @return Grab from ground command
   */
  private Command GrabFromGroundCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.grab_from_intake)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.grab_from_intake))
        .alongWith(m_groundIntake.setIntakePositionCommand(true)
            .andThen(m_groundIntake.grabAlgea())
            .andThen(m_groundIntake.setIntakePositionCommand(false))
            .andThen(m_groundIntake.grabAlgea().andThen(m_algaeArmIntake.grabCommand())))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command for placing coral at the reef.
   * 
   * @param elevatorPosition Elevator position
   * @param armPosition      Arm position
   * @return Place reef coral command
   */
  private Command PlaceReefCoralCommand(ElevatorPosition elevatorPosition, ArmPosition armPosition) {
    return m_elevator.setElevatorPositionCommand(elevatorPosition)
        .alongWith(m_arm.setArmPositionCommand(armPosition))
        .andThen(AutoReleaseCoral());
  }

  private Command AutoReleaseCoral() {
    return m_coralArmIntake.releaseCommand()
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  private Command PlaceAutomaticCoral() {
    return new Command() {
      int currentLevel = 4;
      Command currentCommand = PlaceReefCoralInitCommand(ElevatorPosition.coral_L4, ArmPosition.coral_L4);
      boolean isFinished = false;

      @Override
      public void initialize() {
        currentCommand.schedule();
      }

      @Override
      public void execute() {
        if (currentCommand.isFinished()) {
          if (VisionSubsystem.getLimelightObjectTarget()) {
            currentLevel--;
          }
          else
          {
            isFinished = true;
            return;
          }

          switch (currentLevel) {
            case 3:
              currentCommand = PlaceReefCoralInitCommand(ElevatorPosition.coral_L3, ArmPosition.coral_L23);
              break;
            case 2:
              currentCommand = PlaceReefCoralInitCommand(ElevatorPosition.coral_L2, ArmPosition.coral_L23);
              break;
            case 1:
              currentCommand = PlaceReefCoralInitCommand(ElevatorPosition.coral_L1, ArmPosition.coral_L1);
            default:
              isFinished = true;
              break;
          }
          currentCommand.schedule();
        }
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
    };
  }

  private Command PlaceReefCoralInitCommand(ElevatorPosition elevatorPosition, ArmPosition armPosition) {
    return m_elevator.setElevatorPositionCommand(elevatorPosition)
        .alongWith(m_arm.setArmPositionCommand(armPosition));
  }

  /**
   * Command for grabbing algae at the reef.
   * 
   * @param position Elevator position
   * @return Grab algae reef command
   */
  private Command GrabAlgaeReefCommand(ElevatorPosition position) {
    return m_elevator.setElevatorPositionCommand(position)
        .andThen(m_algaeArmIntake.grabCommand())
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command for dropping algae at the processor.
   * 
   * @return Drop algae processor command
   */
  private Command DropAlgaeProcessorCommand() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.processor)
        .andThen(m_algaeArmIntake.releaseCommand())
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command for grabbing from the intake source.
   * 
   * @return Intake source grab command
   */
  private Command IntakeSourceGrabCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.grabSource)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.source))
        .andThen(m_coralArmIntake.grabCommand())
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  private Command IntakeSourceInitCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.grabSource)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.source));
  }

  /**
   * Command to set all systems to idle state.
   * 
   * @return Idle systems command
   */
  private Command IdleSystemsCommand() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.idle)
        .alongWith(m_arm.setArmPositionCommand(ArmPosition.idle))
        .alongWith(m_groundIntake.setIntakePositionCommand(false))
        .alongWith(m_groundIntake.stopMotors())
        .alongWith(m_coralArmIntake.stopMotors())
        .alongWith(m_algaeArmIntake.stopMotors());
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