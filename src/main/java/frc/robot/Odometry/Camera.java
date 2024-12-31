package frc.robot.Odometry;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Camera {
    private PhotonCamera camera;
    private PhotonCameraSim simCamera;
    private Matrix<N3, N1> singleTagStdDevs;
    private Matrix<N3, N1> multiTagStdDevsMatrix;
    private SimCameraProperties simProperties;
    private PhotonPoseEstimator poseEstimator;
    private Transform3d robotToCam;

    public Camera(String name, Translation3d robotToCamTranslation, Rotation3d robotToCamRotation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
        camera = new PhotonCamera(name);

        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevsMatrix = multiTagStdDevsMatrix;

        simProperties = new SimCameraProperties();
        simProperties.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        simProperties.setCalibError(0.25, 0.08);
        simProperties.setFPS(30);
        simProperties.setAvgLatencyMs(35);
        simProperties.setLatencyStdDevMs(5);

        simCamera = new PhotonCameraSim(camera, simProperties);
        robotToCam = new Transform3d(robotToCamTranslation, robotToCamRotation);

        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public PhotonCameraSim getSimCamera() {
        return simCamera;
    }

    public Matrix<N3, N1> getSingleTagStdDevs() {
        return singleTagStdDevs;
    }

    public Matrix<N3, N1> getMultiTagStdDevsMatrix() {
        return multiTagStdDevsMatrix;
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public SimCameraProperties getSimProperties() {
        return simProperties;
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    public Optional<EstimatedRobotPose> updatePoseEstimator() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            return poseEstimator.update(results.get(results.size() - 1));
        }
        return Optional.empty();
    }
}
