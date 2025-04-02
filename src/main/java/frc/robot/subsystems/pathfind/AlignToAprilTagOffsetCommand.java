package frc.robot.subsystems.pathfind;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class AlignToAprilTagOffsetCommand extends Command {
    private final DriveSubsystem swerve;
    private final PIDController xController = new PIDController(5, 0, 0);
    private final PIDController yController = new PIDController(5, 0, 0);
    private final PIDController thetaController = new PIDController(5.0, 0, 0);
    private String alignType;

    public AlignToAprilTagOffsetCommand(DriveSubsystem swerve, String alignType) {
        this.swerve = swerve;
        this.alignType = alignType;
        addRequirements(swerve);

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        thetaController.setTolerance(0.05);
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

        if (alignType.equals("human")) {
            tagPose = new Pose3d(
                    DriveSubsystem.fieldPositions.getClosestHumanPose(DriveSubsystem.getRPose3d().toPose2d()));
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                tagPose = new Pose3d(FlippingUtil.flipFieldPose(tagPose.toPose2d()));
            }
        }
        else if(alignType.equals("net"))
        {
            tagPose = new Pose3d(new Pose2d(new Translation2d(7.596, 6.551), Rotation2d.fromDegrees(0)));
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                tagPose = new Pose3d(FlippingUtil.flipFieldPose(tagPose.toPose2d()));
            }
        }

        Translation3d tagTranslation = tagPose.getTranslation();
        Rotation3d tagRotation = tagPose.getRotation();

        // Calculate desired camera position with offset
        //CAMERA DISTANCE: 0.24M
        double forwardOffset = (alignType.contains("net") || alignType.contains("human")) ? 0 : 0.435; // 0.5m in front of the tag for non-human alignments
        double lateralOffset = alignType.contains("reef") ? 0 
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

        Logger.recordOutput("Align/Error_X", errorX);
        Logger.recordOutput("Align/Error_Y", errorY);
        Logger.recordOutput("Align/Error_Yaw", yawError);

        Logger.recordOutput("Align/X_Setpoint", xController.atSetpoint());
        Logger.recordOutput("Align/Y_Setpoint", yController.atSetpoint());
        Logger.recordOutput("Align/Yaw_Setpoint", thetaController.atSetpoint());

        // PID speed commands
        double forwardSpeed = yController.calculate(errorY, 0); // Forward/backward
        double strafeSpeed = xController.calculate(errorX, 0); // Left/right
        double thetaSpeed = thetaController.calculate(yawError, 0); // rotation

        thetaSpeed = Math.toRadians(thetaSpeed);
        // Use gyro heading for field-relative drive
        Rotation2d robotHeading = DriveSubsystem.getHeading();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-strafeSpeed, -forwardSpeed, -thetaSpeed * 10,
                robotHeading);
        swerve.setSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}