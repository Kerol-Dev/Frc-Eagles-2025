package frc.robot.Subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
// import frc.robot.Odometry.Camera;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Odometry.PoseEstimator;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private PoseEstimator poseEstimator;
    private Notifier odometryThread;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Swerve");
    StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct)
            .publish();
    StructPublisher<Pose2d> simDriveTrainPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("Simulation DriveTrain Pose Pub", Pose2d.struct).publish();

    public SwerveSubsystem(File dir) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(dir).createSwerveDrive(SwerveDriveConstants.maximumDriveSpeed);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        setHeadingCorrection(false);
        swerveDrive.chassisVelocityCorrection = true;
        swerveDrive.autonomousChassisVelocityCorrection = true;
        swerveDrive.setAngularVelocityCompensation(true, true, 0.13);
        swerveDrive.setModuleEncoderAutoSynchronize(true, 0.25);
        swerveDrive.setCosineCompensator(Robot.isReal());

        poseEstimator = new PoseEstimator(() -> swerveDrive.getSimulationDriveTrainPose().get());

        // poseEstimator.addCamera(
        //         new Camera("left",
        //                 new Translation3d(Units.inchesToMeters(12.056),
        //                         Units.inchesToMeters(10.981),
        //                         Units.inchesToMeters(8.44)),
        //                 new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
        //                 VecBuilder.fill(4, 4, 8),
        //                 VecBuilder.fill(0.5, 0.5, 1)));

        // poseEstimator.addCamera(
        //         new Camera("right",
        //                 new Translation3d(Units.inchesToMeters(12.056),
        //                         Units.inchesToMeters(10.981),
        //                         Units.inchesToMeters(8.44)),
        //                 new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
        //                 VecBuilder.fill(4, 4, 8),
        //                 VecBuilder.fill(0.5, 0.5, 1)));

        // stop the yagsl odometry thread
        swerveDrive.stopOdometryThread();
        odometryThread = new Notifier(this::updateOdometry);
        odometryThread.setName("Odometry Thread");
        odometryThread.startPeriodic(Robot.isSimulation() ? 0.01 : 0.02);

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(this::getPose, this::resetOdometry, this::getRobotVelocity, this::setChasisSpeeds,
                    new PPHolonomicDriveController(SwerveDriveConstants.PPDrive_PID, SwerveDriveConstants.PPRotation_PID), config, () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            System.out.println("Error Config");
        }
    }

    private Translation2d applyDeadband(double vX, double vY) {
        if (Math.abs(vX) < 0.2 && Math.abs(vY) < 0.2) {
            vX = 0;
            vY = 0;
        }
        return new Translation2d(vX, vY);
    }

    /**
     * The primary method for controlling the drivebase. Takes a Translation2d and a
     * rotation rate, and calculates and commands module states accordingly.
     * Can use either open-loop or closed-loop velocity control for the wheel
     * velocities. Also has field- and robot-relative modes,
     * which affect how the translation vector is used.
     * 
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per second.
     *                      In robot-relative mode, positive x is torwards the bow
     *                      (front) and positive y is torwards port (left).
     *                      In field-relative mode, positive x is away from the
     *                      alliance wall (field North) and positive y is torwards
     *                      the left wall when looking
     *                      through the driver station glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot relativity.
     * @param feildRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true
     *                      to disable closed-loop.
     * @return The Drive Command
     */
    public Command drive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, DoubleSupplier throttle,
            boolean feildRelative, boolean openLoop) {
        return run(() -> {
            Translation2d input = applyDeadband(vX.getAsDouble(), vY.getAsDouble());

            double speedMultiplier = MathUtil.clamp(throttle.getAsDouble(), 0.1, 1);

            double xVelocity = (input.getX() * getMaxSpeed()) * speedMultiplier;
            double yVelocity = (input.getY() * getMaxSpeed()) * speedMultiplier;
            Translation2d translation = new Translation2d(xVelocity, yVelocity);

            swerveDrive.drive(translation, omega.getAsDouble(), feildRelative, openLoop, SwerveDriveConstants.rotationAxis);
        }).withName("Drive");
    }

    /**
     * A wrapper for driving the robot in teleop
     * 
     * @param vX       translational velocity along the X axis field relative
     * @param vY       translational velocity along the Y axis field relative
     * @param omega    Rotational Velocity CCW+
     * @param throttle Speed limiter 0 - 1
     * @return The Drive Command
     */
    public Command teleopDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, DoubleSupplier throttle) {
        return drive(vX, vY, () -> (Math.pow(MathUtil.applyDeadband(omega.getAsDouble(), 0.2), 3)
                * getSwerveController().config.maxAngularVelocity) * 0.6, throttle, true, false);
    }

    public Command centerModules() {
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(module -> module.setAngle(0)));
    }

    public Command lockPose() {
        return Commands.run(() -> swerveDrive.lockPose(), this);
    }

    // test command to make sure swerve is odometry is oriented the correct way
    // should spin the robot counter clockwise
    // if not read the docs
    // https://yagsl.gitbook.io/yagsl/configuring-yagsl/the-eight-steps
    public Command spinCounterClockwise() {
        return run(() -> setChasisSpeeds(new ChassisSpeeds(0, 0, 1)));
    }

    /**
     * Command to characterize the robot drive motors using SysId
     * 
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12), 3, 5, 3);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     * 
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest
                .generateSysIdCommand(SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3, 5, 3);
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     * 
     * @param speeds Chassis speeds to set.
     */
    private void setChasisSpeeds(ChassisSpeeds speeds) {
        swerveDrive.setChassisSpeeds(speeds);
    }

    private void updateOdometry() {
        swerveDrive.updateOdometry();
        poseEstimator.updatePoseEstimation(swerveDrive);
    }

    /**
     * Gets the max speed of swerve drive
     * 
     * @return The max speed of the robot in Meters/s
     */
    public double getMaxSpeed() {
        return SwerveDriveConstants.maximumDriveSpeed;
    }

    /**
     * Gets the point that the robot rotates around
     * 
     * @return The Robots Center of rotation as a {@link Translation2d}
     */
    public Translation2d getRotationCenter() {
        return SwerveDriveConstants.rotationAxis;
    }

    /**
     * Sets the center of rotation the robot will rotate around
     * 
     * @param rotationCenter the point the robot will rotate around as a
     *                       {@link Translation2d}
     */
    public void setRotationCenter(Translation2d rotationCenter) {
        SwerveDriveConstants.rotationAxis = rotationCenter;
    }

    /**
     * Gets the current gyro {@link Rotation3d} of the robot, as reported by the imu
     * 
     * @return The heading as a {@link Rotation3d} angle
     */
    public Rotation3d getGyroRotation() {
        return swerveDrive.getGyroRotation3d();
    }

    public void setHeadingCorrection(boolean headingCorrection) {
        swerveDrive.setHeadingCorrection(headingCorrection);
    }

    public boolean getHeadingCorrection() {
        return swerveDrive.headingCorrection;
    }

    /**
     * Gets the current swerve drive kinematics
     * 
     * @return the current state of the swerve drive kinematics as
     *         {@link SwerveDriveKinematics}
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Gets the current module positions (azimuth and wheel position (meters)).
     * Inverts the distance from each module if invertOdometry is true.
     * 
     * @return A list of {@link SwerveModulePosition}s containg the current module
     *         positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return swerveDrive.getModulePositions();
    }

    /**
     * Resets odometry to the given pose
     * 
     * @param pose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Set the expected gyroscope angle using a {@link Rotation3d} object. To reset
     * gyro,
     * set to a new {@link Rotation3d} subtracted from the current gyroscopic
     * readings {@link SwerveIMU#getRotation3d()}.
     * 
     * @param rotation
     */
    public void setRotation(double rotation) {
        swerveDrive.setGyro(new Rotation3d(0, 0, rotation));
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     * 
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Post the trajectory to the field
     * 
     * @param trajectory the trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     * 
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the imu. CCW
     * positive, not wrapped.
     * 
     * @return The yaw as a {@link Rotation2d} angle
     */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     * 
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Gets the current roll angle of the robot, as reported by the imu.
     * 
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getRoll() {
        return swerveDrive.getRoll();
    }

    /**
     * Gets the current module states (azimuth and velocity)
     * 
     * @return list of {@link SwerveModuleState} containing the current module
     *         states
     */
    public SwerveModuleState[] getStates() {
        return swerveDrive.getStates();
    }

    public SwerveModuleState[] getDesiredStates() {
        double[] raw = SwerveDriveTelemetry.desiredStates;
        SwerveModuleState FL = new SwerveModuleState(raw[1], Rotation2d.fromDegrees(raw[0]));
        SwerveModuleState FR = new SwerveModuleState(raw[3], Rotation2d.fromDegrees(raw[2]));
        SwerveModuleState BL = new SwerveModuleState(raw[5], Rotation2d.fromDegrees(raw[4]));
        SwerveModuleState BR = new SwerveModuleState(raw[7], Rotation2d.fromDegrees(raw[6]));
        return new SwerveModuleState[] { FL, FR, BL, BR };
    }

    public ChassisSpeeds getDesiredSpeeds() {
        double[] raw = SwerveDriveTelemetry.desiredChassisSpeeds;
        return new ChassisSpeeds(raw[0], raw[1], Math.toRadians(raw[2]));
    }

    public SwerveModule getFrontLeftModule() {
        return swerveDrive.getModuleMap().get("frontleft");
    }

    public SwerveModule getFrontRightModule() {
        return swerveDrive.getModuleMap().get("frontright");
    }

    public SwerveModule getBackLeftModule() {
        return swerveDrive.getModuleMap().get("backleft");
    }

    public SwerveModule getBackRightModule() {
        return swerveDrive.getModuleMap().get("backright");
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick [-1,1] and an
     * angle.
     *
     * @param xInput                     X joystick input for the robot to move in
     *                                   the X direction. X = xInput * maxSpeed
     * @param yInput                     Y joystick input for the robot to move in
     *                                   the Y direction. Y = yInput *
     *                                   maxSpeed;
     * @param angle                      The desired angle of the robot in radians.
     * @param currentHeadingAngleRadians The current robot heading in radians.
     * @param maxSpeed                   Maximum speed in meters per second.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double angle, double currentHeadingAngleRadians,
            double maxSpeed) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle, currentHeadingAngleRadians,
                maxSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Gets the Swerve controller for controlling heading of the robot.
     * 
     * @return The swerve Controller as a {@link SwerveController}
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Gets the current swerve drive configuration
     * 
     * @return The swerve Drive configuration as a {@link SwerveDriveConfiguration}
     */
    public SwerveDriveConfiguration getConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public double getMaximumAngularVelocity() {
        return swerveDrive.getMaximumChassisAngularVelocity();
    }

    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(2.4, 4.7, Rotation2d.fromDegrees(180)), Timer.getFPGATimestamp());
    }

    @Override
    public void periodic() {
        posePub.set(getPose());
    }

    @Override
    public void simulationPeriodic() {
        simDriveTrainPosePub.set(swerveDrive.getSimulationDriveTrainPose().get());
    }
}
