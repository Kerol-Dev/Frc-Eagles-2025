package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Camera {
    public String cameraName = "photon_camera";
    public PhotonCamera cameraObject;
    public Transform3d robotToCamPos;
    private final PoseStrategy poseStrategy;
    private final PhotonPoseEstimator poseEstimator;

    public Camera(String name, Translation3d pos, Rotation3d rot) {
        cameraName = name;
        cameraObject = new PhotonCamera(cameraName);
        robotToCamPos = new Transform3d(pos, rot);
        poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
                poseStrategy, robotToCamPos);
    }

    public EstimatedRobotPose update()
    {
        List<PhotonPipelineResult> results = cameraObject.getAllUnreadResults();
        return poseEstimator.update(results.get(results.size() - 1)).get();
    }
}