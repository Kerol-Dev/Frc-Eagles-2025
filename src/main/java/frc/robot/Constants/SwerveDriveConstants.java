package frc.robot.Constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveConstants {
    public static final double maximumDriveSpeed = 4.8; // m/s
    public static Translation2d rotationAxis = new Translation2d(); //Axis To Rotate On
    public static final PIDConstants PPDrive_PID = new PIDConstants(5, 0, 0); // Pathplanner Movement PID
    public static final PIDConstants PPRotation_PID = new PIDConstants(5, 0, 0); // Pathplanner Rotation PID
}
