package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
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

@AutoLog
public class VisionSubsystem extends SubsystemBase {
    /** Pose estimator for swerve drive using vision data */
    SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            DriveSubsystem.getHeading(), DriveSubsystem.getModulePositions(), new Pose2d());

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
        m_poseEstimator.update(
                DriveSubsystem.getHeading(),
                DriveSubsystem.getModulePositions());

        if (Robot.isSimulation())
            return;

        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if (useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        } else if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation("limelight",
                    m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (Math.abs(DriveSubsystem.m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees
                                                                 // per second,
            // ignore vision updates
            {
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
        }
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