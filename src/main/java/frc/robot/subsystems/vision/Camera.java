package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

public class Camera {
    public String cameraName = "photon_camera";
    public PhotonCamera cameraObject;
    public Transform3d robotToCamPos;
    private final PoseStrategy poseStrategy;
    public final PhotonPoseEstimator poseEstimator;

    public Camera(String name, Translation3d pos, Rotation3d rot) {
        cameraName = name;
        cameraObject = new PhotonCamera(cameraName);
        robotToCamPos = new Transform3d(pos, rot);
        poseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
        poseEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout,
                poseStrategy, robotToCamPos);
    }

    public EstimatedRobotPose update() {
        // Update Pose Estimation Per Camera
        List<PhotonPipelineResult> results = cameraObject.getAllUnreadResults();
        if(results.size() < 1 || results.isEmpty()) {
            return null;
        }

        Optional<EstimatedRobotPose> rOptional = poseEstimator.update(results.get(results.size() - 1));
        return rOptional.isEmpty() ? null : rOptional.get();
    }
}