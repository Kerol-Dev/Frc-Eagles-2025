package frc.robot.subsystems.pathfind;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
    private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
    private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
    private static final LoggedTunableNumber driveMaxVelocity = new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
    private static final LoggedTunableNumber driveMaxVelocityTop = new LoggedTunableNumber(
            "DriveToPose/DriveMaxVelocityTop");
    private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber(
            "DriveToPose/DriveMaxAcceleration");
    private static final LoggedTunableNumber driveMaxAccelerationTop = new LoggedTunableNumber(
            "DriveToPose/DriveMaxAccelerationTop");
    private static final LoggedTunableNumber driveMaxVelocityAuto = new LoggedTunableNumber(
            "DriveToPose/DriveMaxVelocityAuto");
    private static final LoggedTunableNumber driveMaxVelocityAutoTop = new LoggedTunableNumber(
            "DriveToPose/DriveMaxVelocityAutoTop");
    private static final LoggedTunableNumber driveMaxAccelerationAuto = new LoggedTunableNumber(
            "DriveToPose/DriveMaxAccelerationAuto");
    private static final LoggedTunableNumber driveMaxAccelerationAutoTop = new LoggedTunableNumber(
            "DriveToPose/DriveMaxAccelerationAutoTop");
    private static final LoggedTunableNumber thetaMaxVelocity = new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
    private static final LoggedTunableNumber thetaMaxVelocityTop = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxVelocityTop");
    private static final LoggedTunableNumber thetaMaxAcceleration = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxAcceleration");
    private static final LoggedTunableNumber thetaMaxAccelerationTop = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxAccelerationTop");
    private static final LoggedTunableNumber thetaMaxVelocityAuto = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxVelocityAuto");
    private static final LoggedTunableNumber thetaMaxVelocityAutoTop = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxVelocityAutoTop");
    private static final LoggedTunableNumber thetaMaxAccelerationAuto = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxAccelerationAuto");
    private static final LoggedTunableNumber thetaMaxAccelerationAutoTop = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxAccelerationAutoTop");
    private static final LoggedTunableNumber driveTolerance = new LoggedTunableNumber("DriveToPose/DriveTolerance");
    private static final LoggedTunableNumber thetaTolerance = new LoggedTunableNumber("DriveToPose/ThetaTolerance");
    private static final LoggedTunableNumber linearFFMinRadius = new LoggedTunableNumber(
            "DriveToPose/LinearFFMinRadius");
    private static final LoggedTunableNumber linearFFMaxRadius = new LoggedTunableNumber(
            "DriveToPose/LinearFFMaxRadius");
    private static final LoggedTunableNumber thetaFFMinError = new LoggedTunableNumber("DriveToPose/ThetaFFMinError");
    private static final LoggedTunableNumber thetaFFMaxError = new LoggedTunableNumber("DriveToPose/ThetaFFMaxError");
    private static final LoggedTunableNumber setpointMinVelocity = new LoggedTunableNumber(
            "DriveToPose/SetpointMinVelocity");
    private static final LoggedTunableNumber minDistanceVelocityCorrection = new LoggedTunableNumber(
            "DriveToPose/MinDistanceVelocityCorrection");
    private static final LoggedTunableNumber minLinearFFSReset = new LoggedTunableNumber(
            "DriveToPose/MinLinearFFSReset");
    private static final LoggedTunableNumber minThetaFFSReset = new LoggedTunableNumber("DriveToPose/MinThetaFFSReset");

    static {
        drivekP.initDefault(0.05);
        drivekD.initDefault(0.0);
        thetakP.initDefault(3.0);
        thetakD.initDefault(0.0);

        driveMaxVelocity.initDefault(3.5);
        driveMaxVelocityTop.initDefault(3.5);
        driveMaxAcceleration.initDefault(3);
        driveMaxAccelerationTop.initDefault(3);
        driveMaxVelocityAuto.initDefault(4.0);
        driveMaxVelocityAutoTop.initDefault(4.0);
        driveMaxAccelerationAuto.initDefault(3.5);
        driveMaxAccelerationAutoTop.initDefault(1.5);

        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxVelocityTop.initDefault(Units.degreesToRadians(200.0));
        thetaMaxAcceleration.initDefault(8.0);
        thetaMaxAccelerationTop.initDefault(6.0);
        thetaMaxVelocityAuto.initDefault(Units.degreesToRadians(360.0));
        thetaMaxVelocityAutoTop.initDefault(Units.degreesToRadians(200.0));
        thetaMaxAccelerationAuto.initDefault(8.0);
        thetaMaxAccelerationAutoTop.initDefault(6.0);

        driveTolerance.initDefault(0.02);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
        linearFFMinRadius.initDefault(0.01);
        linearFFMaxRadius.initDefault(0.05);
        thetaFFMinError.initDefault(0.0);
        thetaFFMaxError.initDefault(0.0);

        setpointMinVelocity.initDefault(-0.5);
        minDistanceVelocityCorrection.initDefault(0.01);
        minLinearFFSReset.initDefault(0.2);
        minThetaFFSReset.initDefault(0.1);
    }

    private final DriveSubsystem drive;
    private Pose2d target;

    private TrapezoidProfile driveProfile;
    private final PIDController driveController = new PIDController(0.0, 0.0, 0.0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private Translation2d lastSetpointVelocity = Translation2d.kZero;
    private Rotation2d lastGoalRotation = Rotation2d.kZero;
    private double lastTime = 0.0;
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    private boolean running = false;
    private Supplier<Pose2d> robot;

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    public DriveToPose(DriveSubsystem drive) {
        this.drive = drive;
        this.target = new Pose2d();
        addRequirements(drive);
    }

    public DriveToPose(DriveSubsystem drive, Supplier<Pose2d> robot) {
        this(drive);
        this.robot = robot;
    }

    public DriveToPose(
            DriveSubsystem drive,
            Supplier<Pose2d> robot,
            Supplier<Translation2d> linearFF,
            DoubleSupplier omegaFF) {
        this(drive, robot);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
    }

    @Override
    public void initialize() {
        this.robot = drive::getPose;
        this.target = DriveSubsystem.fieldPositions.getRightLeftReef(DriveSubsystem.fieldPositions.getClosestTag(this.robot.get()), true, () -> DriverStation.getAlliance().get() == Alliance.Blue);

        System.out.println(target.getRotation().getDegrees());;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target;
        ChassisSpeeds fieldVelocity = drive.getSpeeds();
        Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
                fieldVelocity.vyMetersPerSecond);

        driveProfile = new TrapezoidProfile(
                DriverStation.isAutonomous()
                        ? new TrapezoidProfile.Constraints(
                                driveMaxVelocityAuto.get(), driveMaxAccelerationAuto.get())
                        : new TrapezoidProfile.Constraints(
                                driveMaxVelocity.get(), driveMaxAcceleration.get()));

        driveController.reset();
        thetaController.reset(
                currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
        lastSetpointVelocity = linearFieldVelocity;
        lastGoalRotation = targetPose.getRotation();
        lastTime = Timer.getTimestamp();
    }

    @Override
    public void execute() {
        running = true;
        

        driveController.setP(0.08);
        driveController.setD(0);
        driveController.setI(0);
        driveController.setTolerance(driveTolerance.get());
        thetaController.setP(0.4);
        thetaController.setD(0);
        thetaController.setTolerance(thetaTolerance.get());

        // Update constraints
        double extensionS = ElevatorSubsystem.elevatorMotor.getPosition().getValueAsDouble() / 4.26;
        driveProfile = new TrapezoidProfile(
                DriverStation.isAutonomous()
                        ? new TrapezoidProfile.Constraints(
                                MathUtil.interpolate(
                                        driveMaxVelocityAuto.get(), driveMaxVelocityAutoTop.get(), extensionS),
                                MathUtil.interpolate(
                                        driveMaxAccelerationAuto.get(),
                                        driveMaxAccelerationAutoTop.get(),
                                        extensionS))
                        : new TrapezoidProfile.Constraints(
                                MathUtil.interpolate(
                                        driveMaxVelocity.get(), driveMaxVelocityTop.get(), extensionS),
                                MathUtil.interpolate(
                                        driveMaxAcceleration.get(), driveMaxAccelerationTop.get(), extensionS)));
        thetaController.setConstraints(
                new TrapezoidProfile.Constraints(
                        DriverStation.isAutonomous()
                                ? MathUtil.interpolate(
                                        thetaMaxVelocityAuto.get(), thetaMaxVelocityAutoTop.get(), extensionS)
                                : MathUtil.interpolate(
                                        thetaMaxVelocity.get(), thetaMaxVelocityTop.get(), extensionS),
                        DriverStation.isAutonomous()
                                ? MathUtil.interpolate(
                                        thetaMaxAccelerationAuto.get(), thetaMaxAccelerationAutoTop.get(), extensionS)
                                : MathUtil.interpolate(
                                        thetaMaxAcceleration.get(), thetaMaxAccelerationTop.get(), extensionS)));

        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target;

        Pose2d poseError = currentPose.relativeTo(targetPose);
        driveErrorAbs = poseError.getTranslation().getNorm();
        thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
        double linearFFScaler = MathUtil.clamp(
                (driveErrorAbs - linearFFMinRadius.get())
                        / (linearFFMaxRadius.get() - linearFFMinRadius.get()),
                0.0,
                1.0);
        double thetaFFScaler = MathUtil.clamp(
                (Units.radiansToDegrees(thetaErrorAbs) - thetaFFMinError.get())
                        / (thetaFFMaxError.get() - thetaFFMinError.get()),
                0.0,
                1.0);

        // Calculate drive velocity
        // Calculate setpoint velocity towards target pose
        var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
        double setpointVelocity = direction.norm() <= minDistanceVelocityCorrection
                .get() // Don't calculate velocity in direction when really close
                        ? lastSetpointVelocity.getNorm()
                        : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
        setpointVelocity = Math.max(setpointVelocity, setpointMinVelocity.get());
        State driveSetpoint = driveProfile.calculate(
                0.02,
                new State(
                        direction.norm(), -setpointVelocity), // Use negative as profile has zero at target
                new State(0.0, 0.0));
        double driveVelocityScalar = driveController.calculate(driveErrorAbs, driveSetpoint.position)
                + driveSetpoint.velocity * linearFFScaler;
        if (driveErrorAbs < driveController.getErrorTolerance())
            driveVelocityScalar = 0.0;
        Rotation2d targetToCurrentAngle = currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
        Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
        lastSetpointTranslation = new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
                .transformBy(GeomUtil.toTransform2d(driveSetpoint.position, 0.0))
                .getTranslation();
        lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

        // Calculate theta speed
        double thetaSetpointVelocity = Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
                ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                        / (Timer.getTimestamp() - lastTime)
                : thetaController.getSetpoint().velocity;

        double thetaVelocity = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
                + thetaController.getSetpoint().velocity * thetaFFScaler;

        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;
        lastGoalRotation = targetPose.getRotation();
        lastTime = Timer.getTimestamp();

        // Scale feedback velocities by input ff
        final double linearS = MathUtil.clamp(linearFF.get().getNorm() * 3.0, 0.0, 1.0);
        final double thetaS = MathUtil.clamp(Math.abs(omegaFF.getAsDouble()) * 3.0, 0.0, 1.0);
        driveVelocity = driveVelocity.interpolate(linearFF.get().times(DriveConstants.kMaxSpeedMetersPerSecond),
                linearS);
        thetaVelocity = MathUtil.interpolate(
                thetaVelocity, omegaFF.getAsDouble() * DriveConstants.kMaxAngularSpeed, thetaS);
        // Reset profiles if enough input
        ChassisSpeeds fieldVelocity = drive.getSpeeds();
        Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
                fieldVelocity.vyMetersPerSecond);
        if (linearS >= minLinearFFSReset.get()) {
            lastSetpointTranslation = currentPose.getTranslation();
            lastSetpointVelocity = linearFieldVelocity;
        }
        if (thetaS >= minThetaFFSReset.get()) {
            thetaController.reset(
                    currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        }

        // Command speeds
        drive.setSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
        Logger.recordOutput(
                "DriveToPose/VelocityMeasured",
                -linearFieldVelocity
                        .toVector()
                        .dot(targetPose.getTranslation().minus(currentPose.getTranslation()).toVector())
                        / driveErrorAbs);
        Logger.recordOutput("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput(
                "DriveToPose/Setpoint",
                new Pose2d[] {
                        new Pose2d(
                                lastSetpointTranslation,
                                Rotation2d.fromRadians(thetaController.getSetpoint().position))
                });
        SmartDashboard.putNumber("X BEBEK", targetPose.getX());
        SmartDashboard.putNumber("Y BEBEK", targetPose.getY());
        SmartDashboard.putNumber("Rot BEBEK", targetPose.getRotation().getDegrees());
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] { targetPose });
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        running = false;
        // Clear logs
        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
    }

    /**
     * Checks if the robot pose is within the allowed drive and theta tolerances.
     */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
                && Math.abs(driveErrorAbs) < driveTolerance
                && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }
}
