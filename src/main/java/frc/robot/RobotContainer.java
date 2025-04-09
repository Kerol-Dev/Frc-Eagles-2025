package frc.robot;

import java.util.function.BooleanSupplier;

// Import necessary libraries and classes
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.misc.ArmPosition;
import frc.robot.subsystems.misc.ElevatorPosition;
import frc.robot.subsystems.pathfind.AlignToAprilTagOffsetCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final IntakeSubsystem m_Intake = new IntakeSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClimbSubsystem m_Climb = new ClimbSubsystem();
  // public final LedSubsystem m_LedSubsystem = new LedSubsystem();

  // Autonomous command chooser
  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Slow speed toggle flag
  public boolean slowSpeedEnabled = false;

  // Object Mode
  public static boolean coralMode = true;
  private boolean climbMode = false;

  // Track button press counts
  private String lastActivePov = "";
  private int activePovPressCount = 0;

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
            -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
            true,
            slowSpeedEnabled, true),
        m_robotDrive));

    m_Intake.setDefaultCommand(m_Intake.grabCommand(false).onlyIf(() -> !m_Intake.getCoralIntakeSensor()));

    // Configure button bindings
    configureButtonBindings();

    // Configure autonomous routines
    configurePathPlanner();

    timerRumble = new Timer();
    timerRumble.start();
  }

  /**
   * Configures PathPlanner autonomous routines.
   */
  private void configurePathPlanner() {
    registerNamedCommand("GrabFromSource", IntakeSourceGrabCommand(),
        () -> !m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("Idle", IdleSystemsCommand(), () -> true, coralMode);

    registerNamedCommand("PlaceL4",
        PlaceReefInit(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l4)),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("PlaceL3",
        PlaceReefInit(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l3)),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("AlignRightL4",
        pathfindToReefL4(true).andThen(PlaceReefInit(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l4))),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("AlignLeftL4",
        pathfindToReefL4(false).andThen(PlaceReefInit(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l4))),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("AlignLeftL3",
        pathfindToReefL3(false).andThen(PlaceReefInit(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l3))),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("AlignRightL3",
        pathfindToReefL3(true).andThen(PlaceReefInit(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l3))),
        () -> m_Intake.getCoralIntakeSensor(), true);

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

    driverController.b().onTrue(checkAndSwitchToCoralMode().andThen(m_Intake.grabCommand(false))
        .onlyIf(() -> !m_Intake.getCoralIntakeSensor()));

    driverController.rightBumper().onTrue(
        checkAndSwitchToCoralMode().andThen(pathfindToReefTeleop(true)).onlyIf(() -> m_Intake.getCoralIntakeSensor()));

    driverController.leftBumper().onTrue(
        checkAndSwitchToCoralMode().andThen(pathfindToReefTeleop(false)).onlyIf(() -> m_Intake.getCoralIntakeSensor()));

    driverController.x()
        .onTrue(m_Intake.releaseCommand(true, ElevatorPosition.grab_algae_reef_1).andThen(IdleSystemsCommand())
            .onlyIf(() -> !climbMode));

    driverController.x().whileTrue(new RunCommand(() -> m_Climb.setClimbSpeed(-1), m_Climb).onlyWhile(() -> climbMode))
        .onFalse(new InstantCommand(() -> m_Climb.stop(), m_Climb));

    driverController.a().whileTrue(new RunCommand(() -> m_Climb.setClimbSpeed(1), m_Climb).onlyIf(() -> climbMode))
        .onFalse(new InstantCommand(() -> m_Climb.stop(), m_Climb));

    driverController.back().onTrue(new InstantCommand(() -> climbMode = !climbMode));

    driverController.povUp()
        .onTrue(new ConditionalCommand(
            PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4, "povUp"),
            PlaceAutomaticNetSequence(), () -> coralMode));
    driverController.povRight()
        .onTrue(PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3, "povRight"));
    driverController.povDown()
        .onTrue(PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l2, ArmPosition.place_coral_l2, "povDown"));
    driverController.povLeft()
        .onTrue(PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l1, ArmPosition.place_coral_l1, "povLeft"));

    driverController.leftTrigger()
        .whileTrue(checkAndSwitchToAlgaeMode().andThen(pathFindToAlgae()
            .alongWith(GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_1, ArmPosition.grab_algae_reef_1))))
        .onFalse(IdleSystemsCommand());

    driverController.rightTrigger()
        .whileTrue(checkAndSwitchToAlgaeMode().andThen(pathFindToAlgae()
            .alongWith(GrabAlgaeReefCommand(ElevatorPosition.grab_algae_reef_2, ArmPosition.grab_algae_reef_2))))
        .onFalse(IdleSystemsCommand());

    driverController.y()
        .onTrue(IdleSystemsCommand().andThen(checkAndSwitchToCoralMode())
            .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())));
  }

  /**
   * Resets all POV button press counts except the specified one.
   * 
   * @param activePov The POV button to exclude from reset
   */
  private void resetPovPressCountsExcept(String activePov) {
    if (!activePov.equals(lastActivePov)) {
      lastActivePov = activePov;
      activePovPressCount = 0;
    }
    activePovPressCount++;
  }

  private Command checkAndSwitchToCoralMode() {
    return new ConditionalCommand(
        SwitchObjectMode().onlyIf(() -> !m_Intake.getAlgaeArmIntakeSensor()),
        new InstantCommand(),
        () -> !coralMode);
  }

  private Command checkAndSwitchToAlgaeMode() {
    return new ConditionalCommand(
        SwitchObjectMode().onlyIf(() -> !m_Intake.getCoralIntakeSensor()),
        new InstantCommand(),
        () -> coralMode);
  }

  private Command PlaceAutomaticNetSequence() {
    return new InstantCommand(() -> resetPovPressCountsExcept("povUp"))
        .andThen(new ConditionalCommand(pathFindToNet(),
            PlaceNetCommand().andThen(new InstantCommand(() -> activePovPressCount = 0)),
            () -> activePovPressCount < 2))
        .onlyIf(() -> m_Intake.getCoralIntakeSensor());
  }

  private Command PlaceNetCommand() {
    return m_elevator.setElevatorPositionCommand(ElevatorPosition.place_algae_net)
        .andThen(m_arm.setArmPositionCommand(ArmPosition.place_algae_net))
        .andThen(m_arm.setArmPositionCommand(ArmPosition.drop_algae_net)
            .alongWith(new SequentialCommandGroup(new WaitCommand(0.25),
                m_Intake.releaseCommand(false, ElevatorPosition.place_algae_net))))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)))
        .andThen(new WaitCommand(0.2))
        .andThen(IdleSystemsCommand());
  }

  private Command PlaceAutomaticReefSequence(ElevatorPosition elevatorPosition,
      ArmPosition armPosition,
      String povName) {
    return new InstantCommand(() -> resetPovPressCountsExcept(povName))
        .andThen(new ConditionalCommand(PlaceReefInit(elevatorPosition, armPosition),
            AutoReleaseCoral(elevatorPosition).andThen(new InstantCommand(() -> activePovPressCount = 0)),
            () -> activePovPressCount < 2))
        .onlyIf(() -> m_Intake.getCoralIntakeSensor());
  }

  // private Command PlaceAutomaticReefSequence(ElevatorPosition elevatorPosition,
  // ArmPosition armPosition,
  // String povName) {
  // return PlaceReefInit(elevatorPosition,
  // armPosition).andThen(AutoReleaseCoral(elevatorPosition))
  // .onlyIf(() -> m_Intake.getCoralIntakeSensor());
  // }

  private Command AutoReleaseCoral(ElevatorPosition position) {
    return m_Intake.releaseCommand(false, position)
        .andThen(new InstantCommand(() -> triggerRumble(0.5)))
        .andThen(new WaitCommand(0.2))
        .andThen(IdleSystemsCommand());
  }

  private Command SwitchObjectMode() {
    return new InstantCommand(() -> {
      coralMode = !coralMode;
      ElevatorSubsystem.setElevatorConfiguration(coralMode);
      if (coralMode) {
        m_elevator.setElevatorPosition(0.4);
      } else {
        m_elevator.setElevatorPosition(1.54);
      }
    }).andThen(new WaitUntilCommand(() -> m_elevator.isElevatorAtPosition())).andThen(IdleSystemsCommand());
  }

  private Command PlaceReefInit(ElevatorPosition elevatorPosition, ArmPosition armPosition) {
    return m_arm.setArmPositionCommand(ArmPosition.ElevatorUp)
        .andThen(m_elevator.setElevatorPositionCommand(elevatorPosition))
        .andThen(m_arm.setArmPositionCommand(armPosition));
  }

  static edu.wpi.first.wpilibj.Timer timerRumble;
  static double lastTime = 0;

  public void periodic() {
    if (timerRumble.get() - lastTime > 0.5 && lastTime > 0) {
      lastTime = 0;
      driverController.setRumble(RumbleType.kBothRumble, 0);
    }
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
   * Command for grabbing from the intake source.
   * 
   * @return Intake source grab command
   */
  public Command IntakeSourceGrabCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.idle)
        .alongWith(m_elevator.setElevatorPositionCommand(ElevatorPosition.idle))
        .andThen(m_Intake.grabCommand(false))
        .andThen(new InstantCommand(() -> m_Intake.setIntakeSpeed(0)))
        .andThen(new InstantCommand(() -> triggerRumble(0.5)));
  }

  /**
   * Command to set all systems to idle state.
   * 
   * @return Idle systems command
   */
  public Command IdleSystemsCommand() {
    return m_arm.setArmPositionCommand(ArmPosition.idle)
        .andThen(m_elevator.setElevatorPositionCommand(ElevatorPosition.idle));
  }

  /**
   * Triggers controller rumble for a specified duration.
   * 
   * @param durationSeconds Duration in seconds
   */

  public static void triggerRumble(double durationSeconds) {
    if (DriverStation.isAutonomousEnabled())
      return;

    if (lastTime == 0) {
      driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
      lastTime = timerRumble.get();
    }
  }

  /**
   * Returns the selected autonomous command.
   * 
   * @return Selected autonomous command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private Command pathfindToReefL4(boolean right) {
    return PlaceReefInit(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
        .alongWith(new AlignToAprilTagOffsetCommand(m_robotDrive, "reef" + (right ? "right" : "left"), false));
  }

  private Command pathfindToReefL3(boolean right) {
    return PlaceReefInit(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
        .alongWith(new AlignToAprilTagOffsetCommand(m_robotDrive, "reef" + (right ? "right" : "left"), false));
  }

  private Command pathfindToReefTeleop(boolean right) {
    return PlaceReefInit(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
        .alongWith(new AlignToAprilTagOffsetCommand(m_robotDrive, "reef" + (right ? "right" : "left"), true)
            .until(() -> !m_Intake.getCoralIntakeSensor()));
  }

  private Command pathFindToAlgae() {
    return new AlignToAprilTagOffsetCommand(m_robotDrive, "algae", true)
        .until(() -> m_Intake.getAlgaeArmIntakeSensor());
  }

  private Command pathFindToNet() {
    return new AlignToAprilTagOffsetCommand(m_robotDrive, "net", true)
        .onlyIf(() -> m_Intake.getAlgaeArmIntakeSensor());
  }
}