package frc.robot.subsystems.pathfind;

import edu.wpi.first.wpilibj2.command.Command;
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
    private final PIDController xController = new PIDController(1.5, 0, 0);
    private final PIDController yController = new PIDController(1.5, 0, 0);
    private final PIDController thetaController = new PIDController(2.0, 0, 0);
    private boolean isRight = false;

    public AlignToAprilTagOffsetCommand(DriveSubsystem swerve, boolean isRight) {
        this.swerve = swerve;
        this.isRight = isRight;
        addRequirements(swerve);

        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
        thetaController.setTolerance(2.0);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV("")) {
            swerve.stop();
            return;
        }

        // Tag pose relative to camera
        Pose3d tagPoseCamSpace = LimelightHelpers.getTargetPose3d_CameraSpace("");
        Translation3d tagTranslation = tagPoseCamSpace.getTranslation();
        Rotation3d tagRotation = tagPoseCamSpace.getRotation();

        // Desired offset relative to tag: 0.164m to the RIGHT, 0.5m in front
        Translation3d offsetInTagSpace = new Translation3d(isRight ? 0.164 : -0.164, 0, -0.5); // right and forward (note: Z is forward)

        // Rotate offset from tag-space to camera-space
        Translation3d rotatedOffset = offsetInTagSpace.rotateBy(tagRotation);

        // Desired camera position relative to the tag
        Translation3d desiredCameraPosition = tagTranslation.plus(rotatedOffset);

        // Calculate error in camera space
        double errorX = desiredCameraPosition.getX();
        // double errorY = desiredCameraPosition.getY();
        double errorZ = desiredCameraPosition.getZ();
        // Yaw correction to face tag
        double yawError = tagRotation.getZ(); // degrees, 0 = facing tag

        Logger.recordOutput("Align/Error_X", errorX);
        Logger.recordOutput("Align/Error_Z", errorZ);
        Logger.recordOutput("Align/Error_Yaw", yawError);

        // PID speed commands
        double forwardSpeed = xController.calculate(errorZ, 0); // Forward/backward
        double strafeSpeed = yController.calculate(errorX, 0);  // Left/right
        double thetaSpeed = thetaController.calculate(yawError, 0); // rotation

        // Convert degrees/sec to radians/sec
        thetaSpeed = Math.toRadians(thetaSpeed);

        // Use gyro heading for field-relative drive
        Rotation2d robotHeading = DriveSubsystem.getHeading();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, -strafeSpeed, thetaSpeed, robotHeading);
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