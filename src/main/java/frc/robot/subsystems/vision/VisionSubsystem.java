package frc.robot.subsystems.vision;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase {
    public static VisionSystemSim visionSystemSim = new VisionSystemSim("Main");

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), new Pose2d());

    StructPublisher<Pose3d> structPublisherFL = NetworkTableInstance.getDefault().getTable("Camera Data")
            .getStructTopic("Camera Estimation FL", Pose3d.struct).publish();
    StructPublisher<Pose3d> structPublisherFR = NetworkTableInstance.getDefault().getTable("Camera Data")
            .getStructTopic("Camera Estimation FR", Pose3d.struct).publish();
    StructPublisher<Pose3d> structPublisherRL = NetworkTableInstance.getDefault().getTable("Camera Data")
            .getStructTopic("Camera Estimation RL", Pose3d.struct).publish();
    StructPublisher<Pose3d> structPublisherRR = NetworkTableInstance.getDefault().getTable("Camera Data")
            .getStructTopic("Camera Estimation RR", Pose3d.struct).publish();

    public VisionSubsystem() {
        InitializeCameras();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (Camera camera : VisionConstants.cameras) {
            for (var change : camera.cameraObject.getAllUnreadResults()) {
                visionEst = camera.poseEstimator.update(change);
                visionEst.ifPresent(
                        est -> {
                            switch (camera.cameraName) {
                                case "FrontLeft":
                                    structPublisherFL.set(est.estimatedPose);
                                    break;
                                case "FrontRight":
                                    structPublisherFR.set(est.estimatedPose);
                                    break;
                                case "RearLeft":
                                    structPublisherRL.set(est.estimatedPose);
                                    break;
                                case "RearRight":
                                    structPublisherRR.set(est.estimatedPose);
                                    break;
                                default:
                                    break;
                            }
                        });
            }
        }
        return visionEst;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            VisionConstants.curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                VisionConstants.curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                VisionConstants.curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return VisionConstants.curStdDevs;
    }

    // Simulation
    public void simulationPeriodic() {
        if (VisionConstants.readSimulationPose.get() != null)
            visionSystemSim.update(VisionConstants.readSimulationPose.get());

        var visionEst = getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    updateEstimationStdDevs(visionEst, visionEst.get().targetsUsed);
                    var estStdDevs = getEstimationStdDevs();

                    poseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
    }

    private void InitializeCameras() {
        try {
            VisionConstants.aprilTagFieldLayout = new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getPath() + "/2025-reefscape.json"));
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Simulation
        VisionConstants.readSimulationPose = NetworkTableInstance.getDefault().getTable("PathPlanner")
                .getStructTopic("targetPose", Pose2d.struct).subscribe(null);

        VisionConstants.simCameraProperties.setCalibration(1280, 960, Rotation2d.fromDegrees(100));
        VisionConstants.simCameraProperties.setFPS(30);
        VisionConstants.simCameraProperties.setAvgLatencyMs(15);
        visionSystemSim.addAprilTags(VisionConstants.aprilTagFieldLayout);

        double cameraX = 0.255;
        double cameraY = 0.255;
        double cameraZ = 0.204;
        double cameraPitch = 35;

        VisionConstants.cameras.add(new Camera("FrontLeft", new Translation3d(cameraX, cameraY, cameraZ),
                new Rotation3d(0.0, Math.toRadians(-cameraPitch), Math.toRadians(45))));
        VisionConstants.cameras.add(new Camera("RearRight", new Translation3d(-cameraX, -cameraY, cameraZ),
                new Rotation3d(0.0, Math.toRadians(-cameraPitch), Math.toRadians(225))));
        VisionConstants.cameras.add(new Camera("RearLeft", new Translation3d(-cameraX, cameraY, cameraZ),
                new Rotation3d(0.0, Math.toRadians(-cameraPitch), Math.toRadians(135))));
        VisionConstants.cameras.add(new Camera("FrontRight", new Translation3d(cameraX, -cameraY, cameraZ),
                new Rotation3d(0.0, Math.toRadians(-cameraPitch), Math.toRadians(315))));
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