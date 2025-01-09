package frc.robot.subsystems.vision;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase {
    AprilTagFieldLayout reefScapeLayout;

    // Photon Vision Cameras
    public ArrayList<Camera> cameras = new ArrayList<>();

    public static VisionSystemSim visionSystemSim = new VisionSystemSim("Main");
    public static SimCameraProperties simCameraProperties = new SimCameraProperties();

    private static final int NUM_TAGS = 22;
    private List<StructPublisher<Pose3d>> tagPublishers = new ArrayList<>();
    {
        for (int i = 1; i <= NUM_TAGS; i++) {
            tagPublishers.add(NetworkTableInstance.getDefault().getTable("TagPoses")
                    .getStructTopic("Tag_" + i, Pose3d.struct).publish());
        }
    }

    private StructSubscriber<Pose2d> readTargetpose = NetworkTableInstance.getDefault().getTable("PathPlanner")
            .getStructTopic("targetPose", Pose2d.struct).subscribe(null);

    StructPublisher<Pose3d> frontRightPublisher = NetworkTableInstance.getDefault()
            .getTable("CamPoses")
            .getStructTopic("FrontRight", Pose3d.struct)
            .publish();
    StructPublisher<Pose3d> frontLeftPublisher = NetworkTableInstance.getDefault()
            .getTable("CamPoses")
            .getStructTopic("FrontLeft", Pose3d.struct)
            .publish();
    StructPublisher<Pose3d> rearRightPublisher = NetworkTableInstance.getDefault()
            .getTable("CamPoses")
            .getStructTopic("RearRight", Pose3d.struct)
            .publish();
    StructPublisher<Pose3d> rearLeftPublisher = NetworkTableInstance.getDefault()
            .getTable("CamPoses")
            .getStructTopic("RearLeft", Pose3d.struct)
            .publish();
    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), new Pose2d());

    private Set<String> visibleAprilTags = new HashSet<>();

    public VisionSubsystem() {
        try {
            reefScapeLayout = new AprilTagFieldLayout(
                    Path.of("src/main/deploy/2025-reefscape.json"));
        } catch (Exception e) {
            e.printStackTrace();
        }

        poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        InitializeCameras();

    }

    @Override
    public void simulationPeriodic() {

        if (DriverStation.isAutonomousEnabled()) {
            publishCamPoses();
            UpdatePoseEstimation();

            visibleAprilTags.clear();
            for (int i = 1; i <= NUM_TAGS; i++) {
                if (visibleAprilTags.contains(String.valueOf(i))) {
                    Pose3d pose = reefScapeLayout.getTagPose(i).orElse(null);
                    if (pose != null) {
                        tagPublishers.get(i - 1).set(pose);
                    }
                } else {
                    tagPublishers.get(i - 1).set(new Pose3d(GetRobotPoseEstimated()));
                }
            }
            // UpdatePoseEstimation();
    
            SmartDashboard.putString("Visible AprilTags", visibleAprilTags.toString());
        }

    }

    public void publishCamPoses() {
        Pose2d robotPose2d = readTargetpose.get(); // Get the current robot pose

        if (robotPose2d == null) {
            return;
        } else {
            System.out.println(robotPose2d);
        }

        var robotPose = new Pose3d(robotPose2d);
        visionSystemSim.update(robotPose);


        frontRightPublisher.set(robotPose.plus(new Transform3d(
                new Translation3d(0.255, -0.255, 0.204),
                new Rotation3d(0.0, Math.toRadians(-35), Math.toRadians(315)))));
        frontLeftPublisher.set(robotPose.plus(new Transform3d(
                new Translation3d(0.255, 0.255, 0.204),
                new Rotation3d(0.0, Math.toRadians(-35), Math.toRadians(45)))));
        rearRightPublisher.set(robotPose.plus(new Transform3d(
                new Translation3d(-0.255, -0.255, 0.204),
                new Rotation3d(0.0, Math.toRadians(-35), Math.toRadians(225)))));
        rearLeftPublisher.set(robotPose.plus(new Transform3d(
                new Translation3d(-0.255, 0.255, 0.204),
                new Rotation3d(0.0, Math.toRadians(-35), Math.toRadians(135)))));
    }

    @Override
    public void periodic() {

    }

    private void InitializeCameras() {
        // Forward, (inverted)Right, Upwards, Roll, Pitch, (inverted)Angle
        cameras.add(new Camera("FrontRight", new Translation3d(0.255, -0.255, 0.204), new Rotation3d(0.0, 0, 0)));
        cameras.add(new Camera("FrontLeft", new Translation3d(0.255, 0.255, 0.204), new Rotation3d(0.0, 0, 0)));
        cameras.add(new Camera("RearRight", new Translation3d(-0.255, -0.255, 0.204), new Rotation3d(0.0, 90, 25)));
        cameras.add(new Camera("RearLeft", new Translation3d(-0.255, 0.255, 0.204), new Rotation3d(0.0, 90, 0)));

        visionSystemSim.addAprilTags(reefScapeLayout);
        simCameraProperties.setCalibration(1280, 960, Rotation2d.fromDegrees(100));
        simCameraProperties.setCalibError(0.25, 0.08);
        simCameraProperties.setFPS(30);
        simCameraProperties.setAvgLatencyMs(15);
        simCameraProperties.setLatencyStdDevMs(5);
    }

    public static boolean getLimelightObjectTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    private void UpdatePoseEstimation() {
        poseEstimator.update(DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions());

        for (Camera camera : cameras) {
            EstimatedRobotPose robotPose = camera.update();

            if (robotPose == null)
                continue;

            for (PhotonTrackedTarget target : robotPose.targetsUsed) {
                visibleAprilTags.add(String.valueOf(target.fiducialId));
            }

            poseEstimator.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), robotPose.timestampSeconds);
        }
    }

    public Pose2d GetRobotPoseEstimated() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose2d) {
        poseEstimator.resetPosition(DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), pose2d);
    }

    public boolean isAprilTagVisible(int id) {
        return visibleAprilTags.contains(String.valueOf(id));
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