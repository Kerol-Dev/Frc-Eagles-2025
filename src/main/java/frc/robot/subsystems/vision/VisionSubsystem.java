package frc.robot.subsystems.vision;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase {
    /** Pose estimator for swerve drive using vision data */
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), new Pose2d());

    /**
     * Constructor initializes the vision system and cameras.
     */
    public VisionSubsystem() {
        InitializeCameras();
    }

    /**
     * Periodic update for simulation, including updating pose estimations.
     */
    public void periodic() {
        poseEstimator.update(DriveSubsystem.getHeading(),
                new SwerveModulePosition[] { DriveSubsystem.m_frontLeft.getPosition(),
                        DriveSubsystem.m_frontRight.getPosition(), DriveSubsystem.m_rearLeft.getPosition(),
                        DriveSubsystem.m_rearRight.getPosition() });

        // if(RobotContainer.autoChooser.getSelected().getName().startsWith("M") && !DriverStation.isTeleopEnabled())
        //     return;

LimelightHelpers.SetRobotOrientation("", DriveSubsystem.getHeading().getDegrees(), 0, 0, 0, 0, 0);
if(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("") != null && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose.getX() != 0)
{
    poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose, LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").timestampSeconds);
}
    }

    /**
     * Initializes the cameras and simulation configurations.
     */
    private void InitializeCameras() {
        try {
            VisionConstants.aprilTagFieldLayout = new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getPath() + "/2025-reefscape.json"));
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    public static boolean getLimelightObjectTarget() {
        return LimelightHelpers.getTV("");
    }

    public Pose2d GetRobotPoseEstimated() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose2d) {
        poseEstimator.resetPosition(DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), pose2d);
    }

    public boolean isAprilTagVisible(int id) {
        return VisionConstants.visibleAprilTags.contains(String.valueOf(id));
    }

    public double getDistanceBetweenPose(Pose2d pose) {
        return PhotonUtils.getDistanceToPose(poseEstimator.getEstimatedPosition(), pose);
    }

    public static LimelightTarget_Detector GetLimelightTarget(String limelightName, int ID) {
        if (LimelightHelpers.getTV(limelightName)) {
            LimelightResults result = LimelightHelpers.getLatestResults(limelightName);

            if (ID == -1)
                return result.targets_Detector[0];

            for (LimelightTarget_Detector detector : result.targets_Detector) {
                if (detector.classID == ID) {
                    return detector;
                }
            }
        }
        return null;
    }
}