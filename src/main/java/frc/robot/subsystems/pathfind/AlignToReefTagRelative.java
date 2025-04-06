package frc.robot.subsystems.pathfind;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem drivebase;
  private int tagID = -1;
  private boolean finishedHorizontalOnce = false;
  private boolean finishedRotationOnce = false;

  public AlignToReefTagRelative(boolean isRightScore, DriveSubsystem drivebase) {
    xController = new PIDController(DriveConstants.X_REEF_ALIGNMENT_P, 0.1, 0); // Vertical movement
    yController = new PIDController(DriveConstants.Y_REEF_ALIGNMENT_P, 0.1, 0); // Horitontal movement
    rotController = new PIDController(DriveConstants.ROT_REEF_ALIGNMENT_P, 0.001, 0.0); // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(DriveConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(DriveConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(DriveConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(DriveConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(
        isRightScore ? DriveConstants.Y_SETPOINT_REEF_ALIGNMENT : -DriveConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(DriveConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = DriveSubsystem.fieldPositions.getClosestTag(drivebase.getPose());
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("")) {
      dontSeeTagTimer.reset();

      LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { tagID });

      Pose3d postions = LimelightHelpers.getBotPose3d_TargetSpace("");
      Logger.recordOutput("post", LimelightHelpers.getBotPose3d_TargetSpace(""));

      double xSpeed = xController.calculate(postions.getX());
      double ySpeed = -yController.calculate(postions.getY());
      double rotValue = -rotController.calculate(postions.getRotation().getZ());

      if (yController.atSetpoint()) {
        finishedHorizontalOnce = true;
      }

      if(rotController.atSetpoint()) {
        finishedRotationOnce = true;
      }

      drivebase.drive(finishedHorizontalOnce ? 0 : 0, finishedRotationOnce ? 0 : 0, rotValue, false, false, true);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(0, 0, 0, false, false, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0, false, false, false);
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {});
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long
    // as it gets a tag in the camera
    return dontSeeTagTimer.hasElapsed(DriveConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(DriveConstants.POSE_VALIDATION_TIME);
  }
}