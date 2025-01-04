package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase {
    AprilTagFieldLayout reefScapeLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    // Photon Vision Cameras
    public ArrayList<Camera> cameras = new ArrayList<>();

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), new Pose2d());

    public VisionSubsystem() {
        poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        InitializeCameras();
    }

    @Override
    public void periodic() {
        UpdatePoseEstimation();
    }

    private void InitializeCameras() {
        // Forward, (inverted)Right, Upwards, Roll, Pitch, (inverted)Angle
        cameras.add(new Camera("FrontRight", new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));
        cameras.add(new Camera("FrontLeft", new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));
        cameras.add(new Camera("RearRight", new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));
        cameras.add(new Camera("RearLeft", new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));
    }

    private void UpdatePoseEstimation() {
        poseEstimator.update(DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions());

        for (Camera camera : cameras) {
            EstimatedRobotPose robotPose = camera.update();
            poseEstimator.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), robotPose.timestampSeconds);
        }
    }

    public Pose2d GetRobotPoseEstimated() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose2d) {
        poseEstimator.resetPosition(DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), pose2d);
    }

    public static LimelightResults GetLimelightTarget(String limelightName, int ID) {
        if (LimelightHelpers.getTV(limelightName)) {
            LimelightResults result = LimelightHelpers.getLatestResults(limelightName);

            if (ID == -1)
                return result;

            for (LimelightTarget_Detector detector : result.targets_Detector) {
                if (detector.classID == ID) {
                    return result;
                }
            }
        }
        return null;
    }
}