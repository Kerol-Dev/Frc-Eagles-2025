package frc.robot.subsystems.pathfind;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class AlignToAprilTagOffsetCommand extends Command {
    private final DriveSubsystem swerve;
    private final PIDController xController = new PIDController(7, 0, 0, 0.0067);
    private final PIDController yController = new PIDController(7, 0, 0, 0.0067);
    private final PIDController thetaController = new PIDController(5, 0, 0, 0.0067);
    private String alignType;
    int id = 0;
    boolean isTeleop = false;

    public AlignToAprilTagOffsetCommand(DriveSubsystem swerve, String alignType, boolean isTeleop) {
        this.swerve = swerve;
        this.alignType = alignType;
        this.isTeleop = isTeleop;
        addRequirements(swerve);

        xController.setTolerance(0.00725);
        yController.setTolerance(0.00725);
        thetaController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void execute() {
        if (!VisionSubsystem.latestVisionDetectionValid() && !alignType.equals("net")) {
            swerve.stop();
            return;
        }

        // Tag pose relative to camera
        Pose3d tagPose = VisionConstants.fieldLayout
                .getTagPose(DriveSubsystem.fieldPositions.getClosestTag(DriveSubsystem.getRPose3d().toPose2d())).get();

        id = DriveSubsystem.fieldPositions.getClosestTag(DriveSubsystem.getRPose3d().toPose2d());
        if (alignType.equals("human")) {
            tagPose = new Pose3d(
                    DriveSubsystem.fieldPositions.getClosestHumanPose(DriveSubsystem.getRPose3d().toPose2d()));
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                tagPose = new Pose3d(FlippingUtil.flipFieldPose(tagPose.toPose2d()));
            }
        } else if (alignType.equals("net")) {
            tagPose = new Pose3d(new Pose2d(new Translation2d(7.939, 6.558), Rotation2d.fromDegrees(180)));
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                tagPose = new Pose3d(FlippingUtil.flipFieldPose(tagPose.toPose2d()));
            }
        }

        Translation3d tagTranslation = tagPose.getTranslation();
        Rotation3d tagRotation = tagPose.getRotation();

        // Calculate desired camera position with offset
        // CAMERA DISTANCE: 0.24M
        double forwardOffset = (alignType.contains("net") || alignType.contains("algae")) ? 0 : 0.455; // 0.5m in front
                                                                                                       // of the tag for
                                                                                                       // non-human
                                                                                                       // alignments
        double lateralOffset = !alignType.contains("reef") ? 0
                : (alignType.contains("right") ? 0.164 : -0.164); // 0.164m to the right or left of the tag
        Translation3d offset = new Translation3d(forwardOffset, lateralOffset, 0);

        // Apply rotation to offset
        Translation3d rotatedOffset = offset.rotateBy(tagRotation);

        // Desired camera position relative to the tag
        Translation3d desiredCameraPosition = tagTranslation.plus(rotatedOffset);

        // Get the robot's current pose
        Pose3d robotPose = DriveSubsystem.getRPose3d();
        Translation3d robotTranslation = robotPose.getTranslation();

        // Calculate error in camera space
        double errorX = desiredCameraPosition.getX() - robotTranslation.getX();
        double errorY = desiredCameraPosition.getY() - robotTranslation.getY();
        // Yaw correction to face tag
        double yawError = tagRotation.getZ() - DriveSubsystem.getHeading().getRadians();

        SmartDashboard.putNumber("Align/Error_X", errorX);
        SmartDashboard.putNumber("Align/Error_Y", errorY);
        SmartDashboard.putNumber("Align/Error_Yaw", yawError);

        SmartDashboard.putBoolean("Align/X_Setpoint", xController.atSetpoint());
        SmartDashboard.putBoolean("Align/Y_Setpoint", yController.atSetpoint());
        SmartDashboard.putBoolean("Align/Yaw_Setpoint", thetaController.atSetpoint());
        SmartDashboard.putBoolean("Align/Get_Target", VisionSubsystem.getLimelightObjectTarget());
        
        // PID speed commands
        double forwardSpeed = yController.calculate(errorY, 0); // Forward/backward
        double strafeSpeed = xController.calculate(errorX, 0); // Left/right
        double thetaSpeed = thetaController.calculate(yawError, 0); // rotation

        if(id != 7 && id != 10 && id != 21 && id != 18)
        {
            strafeSpeed *= 0.7;
        }
        else
        {
            forwardSpeed *= 0.7;
        }
       

        forwardSpeed = MathUtil.clamp(forwardSpeed, -0.8, 0.8);
        strafeSpeed = MathUtil.clamp(strafeSpeed, -0.8, 0.8);

        thetaSpeed = Math.toRadians(thetaSpeed);
        // Use gyro heading for field-relative drive
        Rotation2d robotHeading = DriveSubsystem.getHeading();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-strafeSpeed, -forwardSpeed, thetaSpeed,
                robotHeading);
        swerve.setSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && (MathUtil.isNear(-3.14, thetaController.getError(), 0.004) || MathUtil.isNear(3.14, thetaController.getError(), 0.004) || !isTeleop) && VisionSubsystem.getLimelightObjectTarget()) || RobotContainer.joystickUsed;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}