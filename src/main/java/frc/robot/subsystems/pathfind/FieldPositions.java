package frc.robot.subsystems.pathfind;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class FieldPosition {
    public Pose2d pose;
    public String name;
    public int tag;
    public boolean right;

    public FieldPosition(Pose2d pose, String name, int tag, boolean right) {
        this.pose = pose;
        this.name = name;
        this.tag = tag;
        this.right = right;
    }
}

public class FieldPositions {
    public ArrayList<FieldPosition> fieldPositions = new ArrayList<FieldPosition>();

    public void addFieldPosition(Pose2d pose, String name, int tag, boolean right) {
        fieldPositions.add(new FieldPosition(pose, name, tag, right));
    }

    // Method to flip the blue pose to red pose
    public Pose2d flipBlueToRed(Pose2d bluePose) {
        return FlippingUtil.flipFieldPose(bluePose);
    }

    public Pose2d getRightLeftReef(int tag, boolean right, BooleanSupplier isBlue) {
        for (FieldPosition position : fieldPositions) {
            if (position.tag == tag) {
                if (position.right == right)
                    return isBlue.getAsBoolean() ? position.pose : flipBlueToRed(position.pose);
            }
        }
        return null;
    }

    public double getTagRotation(int tag) {

        for (FieldPosition position : fieldPositions) {
            if (position.tag == tag) {
                return position.pose.getRotation().getDegrees();
            }
        }
        return 0;
    }

    public Pose2d getPose(String name) {
        for (FieldPosition fieldPosition : fieldPositions) {
            if (fieldPosition.name.equals(name)) {
                return getPose(fieldPosition);
            }
        }
        return null;
    }

    public Pose2d getPose(FieldPosition fieldPosition) {
        return DriverStation.getAlliance().get() == Alliance.Blue ? fieldPosition.pose : flipBlueToRed(fieldPosition.pose);
    }

    public Pose2d getClosestReefPose(Pose2d robotPose) {
        Pose2d closestReefPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (FieldPosition fieldPosition : fieldPositions) {
            if (fieldPosition.name.contains("reef")) {
                double distance = PhotonUtils.getDistanceToPose(robotPose, getPose(fieldPosition));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestReefPose = getPose(fieldPosition);
                }
            }
        }
        return closestReefPose;
    }

    public Pose2d getClosestHumanPose(Pose2d robotPose) {
        Pose2d closestHumanPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (FieldPosition fieldPosition : fieldPositions) {
            if (fieldPosition.name.contains("human")) {
                double distance = PhotonUtils.getDistanceToPose(robotPose, getPose(fieldPosition));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestHumanPose = fieldPosition.pose;
                }
            }
        }
        return closestHumanPose;
    }

    public int getClosestTag(Pose2d robotPose) {
        int closestTag = -1;
        double closestDistance = Double.MAX_VALUE;
        for (FieldPosition fieldPosition : fieldPositions) {
            if (fieldPosition.name.contains("tag")) {
                double distance = PhotonUtils.getDistanceToPose(robotPose, fieldPosition.pose);
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestTag = fieldPosition.tag;
                }
            }
        }
        return closestTag;
    }

    public Pose2d getClosestAlgeaPose(Pose2d robotPose) {
        Pose2d closestAlgeaPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (FieldPosition fieldPosition : fieldPositions) {
            if (fieldPosition.name.contains("algea")) {
                double distance = PhotonUtils.getDistanceToPose(robotPose, getPose(fieldPosition));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestAlgeaPose = getPose(fieldPosition);
                }
            }
        }
        return closestAlgeaPose;
    }

    public FieldPositions() {
        addFieldPosition(new Pose2d(3.198, 4.189, Rotation2d.fromDegrees(0)), "reef_a", 18, false);
        addFieldPosition(new Pose2d(3.190, 3.852, Rotation2d.fromDegrees(0)), "reef_b", 18, true);
        addFieldPosition(new Pose2d(3.713, 2.992, Rotation2d.fromDegrees(60)), "reef_c", 17, false);
        addFieldPosition(new Pose2d(3.995, 2.835, Rotation2d.fromDegrees(60)), "reef_d", 17, true);
        addFieldPosition(new Pose2d(4.980, 2.840, Rotation2d.fromDegrees(120)), "reef_e", 22, false);
        addFieldPosition(new Pose2d(5.267, 3.003, Rotation2d.fromDegrees(120)), "reef_f", 22, true);
        addFieldPosition(new Pose2d(5.768, 3.866, Rotation2d.fromDegrees(180)), "reef_g", 21, false);
        addFieldPosition(new Pose2d(5.771, 4.187, Rotation2d.fromDegrees(180)), "reef_h", 21, true);
        addFieldPosition(new Pose2d(5.266, 5.053, Rotation2d.fromDegrees(240)), "reef_i", 20, false);
        addFieldPosition(new Pose2d(4.982, 5.222, Rotation2d.fromDegrees(240)), "reef_j", 20, true);
        addFieldPosition(new Pose2d(3.991, 5.219, Rotation2d.fromDegrees(300)), "reef_k", 19, false);
        addFieldPosition(new Pose2d(3.697, 5.050, Rotation2d.fromDegrees(300)), "reef_l", 19, true);


        addFieldPosition(new Pose2d(3.198, 4.189, Rotation2d.fromDegrees(0)), "reef_a", 7, false);
        addFieldPosition(new Pose2d(3.190, 3.852, Rotation2d.fromDegrees(0)), "reef_b", 7, true);
        addFieldPosition(new Pose2d(3.713, 2.992, Rotation2d.fromDegrees(60)), "reef_c", 8, false);
        addFieldPosition(new Pose2d(3.995, 2.835, Rotation2d.fromDegrees(60)), "reef_d", 8, true);
        addFieldPosition(new Pose2d(4.980, 2.840, Rotation2d.fromDegrees(120)), "reef_e", 9, false);
        addFieldPosition(new Pose2d(5.267, 3.003, Rotation2d.fromDegrees(120)), "reef_f", 9, true);
        addFieldPosition(new Pose2d(5.768, 3.866, Rotation2d.fromDegrees(180)), "reef_g", 10, false);
        addFieldPosition(new Pose2d(5.771, 4.187, Rotation2d.fromDegrees(180)), "reef_h", 10, true);
        addFieldPosition(new Pose2d(5.266, 5.053, Rotation2d.fromDegrees(240)), "reef_i", 11, false);
        addFieldPosition(new Pose2d(4.982, 5.222, Rotation2d.fromDegrees(240)), "reef_j", 11, true);
        addFieldPosition(new Pose2d(3.991, 5.219, Rotation2d.fromDegrees(295)), "reef_k", 6, false);
        addFieldPosition(new Pose2d(3.697, 5.050, Rotation2d.fromDegrees(295)), "reef_l", 6, true);

        addFieldPosition(new Pose2d(1.127, 1.029, Rotation2d.fromDegrees(235)), "human_right", -1, false);
        addFieldPosition(new Pose2d(1.097, 6.984, Rotation2d.fromDegrees(125)), "human_left", -1, false);

        addFieldPosition(new Pose2d(5.973, 0.762, Rotation2d.fromDegrees(270)), "processor", -1, false);

        addFieldPosition(new Pose2d(3.206, 4.035, Rotation2d.fromDegrees(0)), "algea_a", 18, false);
        addFieldPosition(new Pose2d(3.851, 2.917, Rotation2d.fromDegrees(60)), "algea_b", 17, false);
        addFieldPosition(new Pose2d(5.125, 2.919, Rotation2d.fromDegrees(120)), "algea_c", 22, false);
        addFieldPosition(new Pose2d(5.763, 4.017, Rotation2d.fromDegrees(180)), "algea_d", 21, false);
        addFieldPosition(new Pose2d(5.137, 5.124, Rotation2d.fromDegrees(240)), "algea_e", 20, false);
        addFieldPosition(new Pose2d(3.849, 5.128, Rotation2d.fromDegrees(300)), "algea_f", 19, false);

        addFieldPosition(new Pose2d(3.206, 4.035, Rotation2d.fromDegrees(0)), "algea_a", 7, false);
        addFieldPosition(new Pose2d(3.851, 2.917, Rotation2d.fromDegrees(60)), "algea_b", 8, false);
        addFieldPosition(new Pose2d(5.125, 2.919, Rotation2d.fromDegrees(120)), "algea_c", 9, false);
        addFieldPosition(new Pose2d(5.763, 4.017, Rotation2d.fromDegrees(180)), "algea_d", 10, false);
        addFieldPosition(new Pose2d(5.137, 5.124, Rotation2d.fromDegrees(240)), "algea_e", 11, false);
        addFieldPosition(new Pose2d(3.849, 5.128, Rotation2d.fromDegrees(300)), "algea_f", 6, false);

        // Adding field positions for reef AprilTags
        addFieldPosition(new Pose2d(13.474446, 3.3012379999999997, Rotation2d.fromDegrees(30)), "tag_6", 6, false);
        addFieldPosition(new Pose2d(13.890498, 4.0208200000000005, Rotation2d.fromDegrees(0)), "tag_7", 7, false);
        addFieldPosition(new Pose2d(13.474446, 4.740402, Rotation2d.fromDegrees(30)), "tag_8", 8, false);
        addFieldPosition(new Pose2d(12.643358, 4.740402, Rotation2d.fromDegrees(60)), "tag_9", 9, false);
        addFieldPosition(new Pose2d(12.227305999999999, 4.0208200000000005, Rotation2d.fromDegrees(90)), "tag_10", 10, false);
        addFieldPosition(new Pose2d(12.643358, 3.3012379999999997, Rotation2d.fromDegrees(60)), "tag_11", 11, false);
        addFieldPosition(new Pose2d(4.073905999999999, 3.3012379999999997, Rotation2d.fromDegrees(60)), "tag_17", 17, false);
        addFieldPosition(new Pose2d(3.6576, 4.0208200000000005, Rotation2d.fromDegrees(90)), "tag_18", 18, false);
        addFieldPosition(new Pose2d(4.073905999999999, 4.740402, Rotation2d.fromDegrees(60)), "tag_19", 19, false);
        addFieldPosition(new Pose2d(4.904739999999999, 4.740402, Rotation2d.fromDegrees(30)), "tag_20", 20, false);
        addFieldPosition(new Pose2d(5.321046, 4.0208200000000005, Rotation2d.fromDegrees(0)), "tag_21", 21, false);
        addFieldPosition(new Pose2d(4.904739999999999, 3.3012379999999997, Rotation2d.fromDegrees(30)), "tag_22", 22, false);
    }
}
