package frc.robot.subsystems.pathfind;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;

public class AlignToAprilTagOffsetCommand extends Command {
    private final DriveSubsystem swerve;
    private final PIDController xController = new PIDController(5, 0, 0);
    private final PIDController yController = new PIDController(5, 0, 0);
    private final PIDController thetaController = new PIDController(5.0, 0, 0);
    private boolean isRight = false;
    private boolean isAlgae = false;

    public AlignToAprilTagOffsetCommand(DriveSubsystem swerve, boolean isRight, boolean isAlgae) {
        this.swerve = swerve;
        this.isRight = isRight;
        this.isAlgae = isAlgae;
        addRequirements(swerve);

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        thetaController.setTolerance(0.05);
        thetaController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV("")) {
        swerve.stop();
        return;
        }

        // Tag pose relative to camera
        Pose3d tagPose = VisionConstants.fieldLayout.getTagPose(DriveSubsystem.fieldPositions.getClosestTag(DriveSubsystem.getRPose3d().toPose2d())).get();
        Translation3d tagTranslation = tagPose.getTranslation();
        Rotation3d tagRotation = tagPose.getRotation();

        // Calculate desired camera position with offset
        Translation3d offset = new Translation3d(
                0.5, // 0.5m in front of the tag
                isAlgae ? 0 : (isRight ? 0.164 : -0.164), // 0.164m to the right or left of the tag
                0
        );

        // Apply rotation to offset
        Translation3d rotatedOffset = offset.rotateBy(tagRotation);

        // Desired camera position relative to the tag
        Translation3d desiredCameraPosition = tagTranslation.plus(rotatedOffset);

        // Get the robot's current pose
        Pose3d robotPose = DriveSubsystem.getRPose3d();
        Translation3d robotTranslation = robotPose.getTranslation();

        // Calculate error in camera space
        double errorX = desiredCameraPosition.getX() - robotTranslation.getX();
        double errorZ = desiredCameraPosition.getY() - robotTranslation.getY();
        // Yaw correction to face tag
        double yawError = tagRotation.getZ() - DriveSubsystem.getHeading().getRadians();

        Logger.recordOutput("Align/Error_X", errorX);
        Logger.recordOutput("Align/Error_Z", errorZ);
        Logger.recordOutput("Align/Error_Yaw", yawError);

        // PID speed commands
        double forwardSpeed = xController.calculate(errorZ, 0); // Forward/backward
        double strafeSpeed = yController.calculate(errorX, 0); // Left/right
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