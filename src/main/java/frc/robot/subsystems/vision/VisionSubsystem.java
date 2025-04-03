package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase {
    /** Pose estimator for swerve drive using vision data */
    SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), new Pose2d());

    public static double lastVisionUpdatePassTime = 0;

    /**
     * Constructor initializes the vision system and cameras.
     */
    public VisionSubsystem() {
        try {
            String path = Filesystem.getDeployDirectory().getPath() + "/2025-reefscape.json";
            VisionConstants.fieldLayout = new AprilTagFieldLayout(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Periodic update for simulation, including updating pose estimations.
     */
    public void periodic() {
        updateLastVisionUpdatePassTime();

        m_poseEstimator.update(
                DriveSubsystem.getHeading(),
                DriveSubsystem.getModulePositions());

        if (Robot.isSimulation())
            return;

        boolean doRejectUpdate = false;

        LimelightHelpers.SetRobotOrientation("",
                m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        if (Math.abs(DriveSubsystem.m_gyro.getRate()) > 720) {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }

        Logger.recordOutput("Vision/Visible ID Count", LimelightHelpers.getRawFiducials("").length);
    }

    /**
     * Updates the last vision update pass time if the Limelight has a valid target.
     */
    public void updateLastVisionUpdatePassTime() {
        if (LimelightHelpers.getTV("")) {
            lastVisionUpdatePassTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
    }

    /**
     * Gets the time in seconds since the last vision detection.
     * 
     * @return Seconds since the last vision detection.
     */
    public static boolean latestVisionDetectionValid() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - lastVisionUpdatePassTime < 5
                && lastVisionUpdatePassTime > 0;
    }

    public static boolean getLimelightObjectTarget() {
        return LimelightHelpers.getTV("");
    }

    public Pose2d GetRobotPoseEstimated() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose2d) {
        m_poseEstimator.resetPosition(DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), pose2d);
    }

    public double getDistanceBetweenPose(Pose2d pose) {
        return PhotonUtils.getDistanceToPose(m_poseEstimator.getEstimatedPosition(), pose);
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