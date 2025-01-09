package frc.robot.subsystems.pathfind;

import java.util.ArrayList;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class FieldPosition {
    public Pose2d pose;
    public String name;

    public FieldPosition(Pose2d pose, String name) {
        this.pose = pose;
        this.name = name;
    }
}

public class FieldPositions {
    public ArrayList<FieldPosition> fieldPositions = new ArrayList<FieldPosition>();

    public void addFieldPosition(Pose2d pose, String name) {
        fieldPositions.add(new FieldPosition(pose, name));
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
        if(!RobotBase.isReal())
        {
            return fieldPosition.pose;
        }
        return DriverStation.getAlliance().get() == Alliance.Blue ? fieldPosition.pose : mirrorPose(fieldPosition.pose);
    }

    public Pose2d mirrorPose(Pose2d blueAlliancePose) {
        final double FIELD_LENGTH_METERS = 17.548;
        // Mirror the X-coordinate by subtracting it from the field length
        double mirroredX = FIELD_LENGTH_METERS - blueAlliancePose.getX();

        // Y-coordinate remains the same
        double mirroredY = blueAlliancePose.getY();

        // Negate the rotation to mirror the heading
        Rotation2d mirroredRotation = blueAlliancePose.getRotation().unaryMinus();

        // Return the mirrored pose
        return new Pose2d(mirroredX, mirroredY, mirroredRotation);
    }

    public Pose2d getClosestReefPose(Pose2d robotPose) {
        Pose2d closestReefPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (FieldPosition fieldPosition : fieldPositions) {
            if (fieldPosition.name.contains("reef")) {
                double distance = PhotonUtils.getDistanceToPose(robotPose, getPose(fieldPosition));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestReefPose = fieldPosition.pose;
                }
            }
        }
        return closestReefPose;
    }

    public Pose2d getClosestHumanPose(Pose2d robotPose)
    {
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

    public Pose2d getClosestAlgeaPose(Pose2d robotPose)
    {
        Pose2d closestAlgeaPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (FieldPosition fieldPosition : fieldPositions) {
            if (fieldPosition.name.contains("algea")) {
                double distance = PhotonUtils.getDistanceToPose(robotPose, getPose(fieldPosition));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestAlgeaPose = fieldPosition.pose;
                }
            }
        }
        return closestAlgeaPose;
    }

    public FieldPositions() {
        addFieldPosition(new Pose2d(0, 0, new Rotation2d()), "human_right");
        addFieldPosition(new Pose2d(0, 0, new Rotation2d()), "human_left");
        addFieldPosition(new Pose2d(0, 0, new Rotation2d()), "reef_a");
        addFieldPosition(new Pose2d(1, 0, new Rotation2d()), "processor");
    }
}