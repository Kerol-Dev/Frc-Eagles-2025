package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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
        Logger.recordOutput(cameraName, robotToCamPos);
    }
}