package frc.robot.Odometry;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import swervelib.SwerveDrive;

public class PoseEstimator {
    private VisionSystemSim visionSim;
    private ArrayList<Camera> cameras;
    private Supplier<Pose2d> getSimDrivetrainPose;

    public PoseEstimator(Supplier<Pose2d> getPose) {
        visionSim = new VisionSystemSim("Vision Sim");
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
        this.cameras = new ArrayList<Camera>();
        this.getSimDrivetrainPose = getPose;
    }

    public void addCamera(Camera camera) {
        cameras.add(camera);
        visionSim.addCamera(camera.getSimCamera(), camera.getRobotToCam());
    }

    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if (Robot.isSimulation())
            visionSim.update(getSimDrivetrainPose.get());
        for (Camera camera : cameras) {
            Optional<EstimatedRobotPose> estPose = camera.updatePoseEstimator();
            if (estPose.isPresent()) {
                EstimatedRobotPose pose = estPose.get();
                swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
                        getEstimationStdDevs(camera, pose));
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs(Camera camera, EstimatedRobotPose poseEst) {
        var estStdDevs = camera.getSingleTagStdDevs();
        var targets = poseEst.targetsUsed;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = camera.getPoseEstimator().getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += PhotonUtils.getDistanceToPose(poseEst.estimatedPose.toPose2d(), tagPose.get().toPose2d());
        }
        if (numTags == 0) {
            return estStdDevs;
        }
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = camera.getMultiTagStdDevsMatrix();
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }
}
