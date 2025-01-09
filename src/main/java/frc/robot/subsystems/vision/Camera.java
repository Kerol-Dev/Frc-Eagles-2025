package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public EstimatedRobotPose update() {
        // Update Camera Robot Pose
        SmartDashboard.putString(cameraName + " Pose 3D", DriveSubsystem.getRPose3d().plus(robotToCamPos).toString());

        // Update Pose Estimation Per Camera
        List<PhotonPipelineResult> results = cameraObject.getAllUnreadResults();
        if(results.size() < 1) {
            return null;
        }
        return poseEstimator.update(results.get(results.size() - 1)).get();
    }
}