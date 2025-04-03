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
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
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
  private final PivotSubsystem m_arm = new PivotSubsystem();
  private final ClimbSubsystem m_Climb = new ClimbSubsystem();
  public final LedSubsystem m_LedSubsystem = new LedSubsystem();

  // Autonomous command chooser
  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Slow speed toggle flag
  public boolean slowSpeedEnabled = false;

  // Object Mode
  public static boolean coralMode = true;

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

    m_Intake.setDefaultCommand(m_Intake.grabCommand(false));

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

    registerNamedCommand("PlaceL4",
        PlaceReefInit(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l4)),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("PlaceL3",
        PlaceReefInit(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l3)),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("AlignRight",
        pathfindToReef(true).andThen(PlaceReefInit(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l4))),
        () -> m_Intake.getCoralIntakeSensor(), true);

    registerNamedCommand("AlignLeft",
        pathfindToReef(false).andThen(PlaceReefInit(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4)
            .andThen(AutoReleaseCoral(ElevatorPosition.place_coral_l4))
            .andThen(IdleSystemsCommand())),
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

    driverController.b()
        .onTrue(checkAndSwitchToCoralMode().andThen(m_Intake.grabCommand(false).alongWith(pathfindToHuman()))
            .onlyIf(() -> !m_Intake.getCoralIntakeSensor()));

    driverController.rightBumper().onTrue(
        checkAndSwitchToCoralMode().andThen(pathfindToReef(true)).onlyIf(() -> m_Intake.getCoralIntakeSensor()));

    driverController.leftBumper().onTrue(
        checkAndSwitchToCoralMode().andThen(pathfindToReef(false)).onlyIf(() -> m_Intake.getCoralIntakeSensor()));

    driverController.x().onTrue(m_Intake.releaseCommand(true, ElevatorPosition.grab_algae_reef_1));

    driverController.back().onTrue(m_Climb.toggleClimbEngaged());

    driverController.povUp()
        .onTrue(new ConditionalCommand(
            PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l4, ArmPosition.place_coral_l4, "povUp"),
            PlaceAlgaeNetCommand(), () -> coralMode));
    driverController.povRight()
        .onTrue(PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3, "povRight"));
    driverController.povDown()
        .onTrue(PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l2, ArmPosition.place_coral_l2, "povLeft"));
    driverController.povLeft()
        .onTrue(PlaceAutomaticReefSequence(ElevatorPosition.place_coral_l1, ArmPosition.place_coral_l1, "povDown"));

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
        .onTrue(IdleSystemsCommand().andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
            .andThen(checkAndSwitchToCoralMode()));
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

  private Command PlaceAlgaeNetCommand() {
    return pathfindToNet().andThen(m_arm.setArmPositionCommand(ArmPosition.ElevatorUp)
        .andThen(m_elevator.setElevatorPositionCommand(ElevatorPosition.place_algae_net))
        .andThen(m_arm.setArmPositionCommand(ArmPosition.place_algae_net))
        .andThen(m_Intake.releaseCommand(true, ElevatorPosition.place_algae_net))
        .andThen(IdleSystemsCommand()));
  }

  private Command PlaceAutomaticReefSequence(ElevatorPosition elevatorPosition, ArmPosition armPosition,
      String povName) {
    return new InstantCommand(() -> resetPovPressCountsExcept(povName))
        .andThen(new ConditionalCommand(PlaceReefInit(elevatorPosition, armPosition),
            AutoReleaseCoral(elevatorPosition).andThen(new InstantCommand(() -> activePovPressCount = 0)),
            () -> activePovPressCount < 2))
        .onlyIf(() -> m_Intake.getCoralIntakeSensor());
  }

  private Command AutoReleaseCoral(ElevatorPosition position) {
    return m_Intake.releaseCommand(false, position)
        .andThen(new InstantCommand(() -> triggerRumble(0.5)))
        .andThen(IdleSystemsCommand());
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

  /**
   * Returns the name of the currently pressed joystick button.
   * 
   * @return The name of the pressed button, or "None" if no button is pressed.
   */
  public String getPressedJoystickButtonName() {
    if (driverController.a().getAsBoolean())
      return "A";
    if (driverController.b().getAsBoolean())
      return "B";
    if (driverController.x().getAsBoolean())
      return "X";
    if (driverController.y().getAsBoolean())
      return "Y";
    if (driverController.start().getAsBoolean())
      return "Start";
    if (driverController.back().getAsBoolean())
      return "Back";
    if (driverController.leftBumper().getAsBoolean())
      return "Left Bumper";
    if (driverController.rightBumper().getAsBoolean())
      return "Right Bumper";
    if (driverController.leftTrigger().getAsBoolean())
      return "Left Trigger";
    if (driverController.rightTrigger().getAsBoolean())
      return "Right Trigger";
    if (driverController.povUp().getAsBoolean())
      return "POV Up";
    if (driverController.povDown().getAsBoolean())
      return "POV Down";
    if (driverController.povLeft().getAsBoolean())
      return "POV Left";
    if (driverController.povRight().getAsBoolean())
      return "POV Right";
    return "None";
  }

  // Pathfinding Commands
  @SuppressWarnings("unused")
  private Command pathfindToHuman() {
    return new AlignToAprilTagOffsetCommand(m_robotDrive, "human");
  }

  private Command pathfindToNet() {
    return new AlignToAprilTagOffsetCommand(m_robotDrive, "net");
  }

  private Command pathfindToReef(boolean right) {
    return new InstantCommand(() -> resetPovPressCountsExcept("povRight"))
        .andThen(PlaceReefInit(ElevatorPosition.place_coral_l3, ArmPosition.place_coral_l3))
        .andThen(new AlignToAprilTagOffsetCommand(m_robotDrive, "reef" + (right ? "right" : "left")));
  }

  private Command pathFindToAlgae() {
    return new AlignToAprilTagOffsetCommand(m_robotDrive, "algae");
  }
}