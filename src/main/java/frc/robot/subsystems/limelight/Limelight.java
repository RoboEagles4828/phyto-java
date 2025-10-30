package frc.robot.subsystems.limelight;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String limelightName;
    private PoseEstimate mt1;

    public Limelight() {
        limelightName = LimelightConstants.LIMELIGHT_NAME;
    }

    /**
     * Perform Filtering on camera reading.
     */
    public boolean acceptLimelightUpdate() {
        // if we don't have an estimate at all, reject
        if (mt1 == null) {
            return false;
        }

        // if we see multiple tags, reject
        if (mt1.tagCount != 1 || mt1.rawFiducials.length != 1) {
            return false;
        }

        // If ambiguity is too high, reject
        if (mt1.rawFiducials[0].ambiguity > .7) {
            return false;
        }

        // If we're too far from the tag, reject
        if (mt1.rawFiducials[0].distToCamera > 3) {
            return false;
        }

        return true;
    }

    public int getCurrentTagID() {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public PoseEstimate getLimeLightPoseEstimate() {
        if (!acceptLimelightUpdate()) {
            return null;
        }
        return mt1;
    }

    @Override
    public void periodic() {
        mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // todo(ben) - feed limelight pose to drive IFF the update is accepted (probably not done in here, but in
        // robotPeriodic?)

        // Print limelight info to SmartDashboard
        SmartDashboard.putNumber("Limelight Fiducial ID", getCurrentTagID());
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putBoolean("Limelight Update Accepted", acceptLimelightUpdate());
        if (mt1 != null) {
            SmartDashboard.putString("Limelight Pose", mt1.pose.toString());
            SmartDashboard.putNumber("Limelight Timestamp", mt1.timestampSeconds);
        }

    //     if (hasTarget() && acceptLimelightUpdate()) {
    //         // if (targetRight) {
    //             var tagToTargetRight = new Transform3d(
    //                 new Translation3d(
    //                     LimelightConstants.ROBOT_OFFSET_METERS,
    //                     LimelightConstants.REEF_RIGHT_OFFSET,
    //                     0
    //                 ),
    //                 new Rotation3d(0, 0, LimelightConstants.ROBOT_ROTATION)
    //             );
    //         // } else {
    //             var tagToTargetLeft = new Transform3d(
    //                 new Translation3d(
    //                     LimelightConstants.ROBOT_OFFSET_METERS,
    //                     LimelightConstants.REEF_LEFT_OFFSET,
    //                     0
    //                 ),
    //                 new Rotation3d(0, 0, LimelightConstants.ROBOT_ROTATION)
    //             );
    //         // }
            
    //         int tagID = getCurrentTagID();

    //         Pose3d aprilTagPos = LimelightConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get();
    //         Pose2d targetPosRight = aprilTagPos.transformBy(tagToTargetRight).toPose2d();
    //         Pose2d targetPosLeft = aprilTagPos.transformBy(tagToTargetLeft).toPose2d();

    //         SmartDashboard.putString("Right Target Pose", targetPosRight.toString());
    //         SmartDashboard.putString("Left Target Pose", targetPosLeft.toString());

    //     }
    }
}
