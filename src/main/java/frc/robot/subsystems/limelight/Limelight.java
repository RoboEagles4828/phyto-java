package frc.robot.subsystems.limelight;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.util.Util4828;

public class Limelight extends SubsystemBase {
    
    private CommandSwerveDrivetrain drivetrain; //< reference to the robot's drivetrain
    private Field2d field;

    private PoseEstimate poseEstimate = null; //< most recent pose estimate from limelight; null if no estimate can be made.
    private boolean isPoseEstimateAcceptable = false; //< if the current poseEstimate should be used.

    private static final String NT_USE_VISION = "UseVision";
    private static final String NT_LEFT_X_OFFSET = "LeftXOffset";
    private static final String NT_LEFT_Y_OFFSET = "LeftYOffset";
    private static final String NT_RIGHT_X_OFFSET = "RightXOffset";
    private static final String NT_RIGHT_Y_OFFSET = "RightYOffset";
    private final NetworkTable debugTable = NetworkTableInstance.getDefault().getTable(Constants.NT_DEBUG);
    private final BooleanSubscriber useVisionToggle = debugTable.getBooleanTopic(NT_USE_VISION).subscribe(true);
    private final DoubleSubscriber leftXOffset = debugTable.getDoubleTopic(NT_LEFT_X_OFFSET).subscribe(0.0);
    private final DoubleSubscriber leftYOffset = debugTable.getDoubleTopic(NT_LEFT_Y_OFFSET).subscribe(-0.265);
    private final DoubleSubscriber rightXOffset = debugTable.getDoubleTopic(NT_RIGHT_X_OFFSET).subscribe(0.0);
    private final DoubleSubscriber rightYOffset = debugTable.getDoubleTopic(NT_RIGHT_Y_OFFSET).subscribe(0.072);

    public Limelight(CommandSwerveDrivetrain drivetrain, Field2d field) {
        this.drivetrain = drivetrain;
        this.field = field;

        debugTable.getBooleanTopic(NT_USE_VISION).publish().setDefault(true);
        debugTable.getDoubleTopic(NT_LEFT_X_OFFSET).publish().setDefault(0.1651);
        debugTable.getDoubleTopic(NT_LEFT_Y_OFFSET).publish().setDefault(-0.15);
        debugTable.getDoubleTopic(NT_RIGHT_X_OFFSET).publish().setDefault(0.1651);
        debugTable.getDoubleTopic(NT_RIGHT_Y_OFFSET).publish().setDefault(0.15);
    }

    public boolean isPoseEstimateAcceptable() {
        return isPoseEstimateAcceptable;
    }

    public int getCurrentTagID() {
        return (int) LimelightHelpers.getFiducialID(LimelightConstants.LIMELIGHT_NAME);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_NAME);
    }

    // Checks if a pose estimate (limelight reading) is of sufficient quality to be used.
    final static double AMBIGUITY_THRESHOLD = 0.7; //< Reject the pose if ambiguity is above this.
    final static double DISTANCE_THRESHOLD = 3.0; //< Reject the pose if distance is above this (meters). 
    private static boolean verifyPoseEstimate(PoseEstimate pose) {
        // if we don't have an estimate at all, reject
        if (pose == null) {
            return false;
        }

        // if we see multiple tags, reject
        if (pose.tagCount != 1 || pose.rawFiducials.length != 1) {
            return false;
        }

        // If ambiguity is too high, reject
        if (pose.rawFiducials[0].ambiguity > AMBIGUITY_THRESHOLD) {
            return false;
        }

        // If we're too far from the tag, reject
        if (pose.rawFiducials[0].distToCamera > DISTANCE_THRESHOLD) {
            return false;
        }

        return true;
    }

    public double getLeftXOffset() { return leftXOffset.get(); }
    public double getLeftYOffset() { return leftYOffset.get(); }
    public double getRightXOffset() { return rightXOffset.get(); }
    public double getRightYOffset() { return rightYOffset.get(); }

    @Override
    public void periodic() {
        // feed the robot's current rotation to the limelight (required for MegaTag2 algorithm)
        LimelightHelpers.SetRobotOrientation(LimelightConstants.LIMELIGHT_NAME, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
        // get the latest pose estimate (using MegaTag2) from the drivetrain and check its quality
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LIMELIGHT_NAME);
        isPoseEstimateAcceptable = verifyPoseEstimate(poseEstimate);
        
        // feed vision estimate to the drivetrain, if the reading is good.
        // todo(ben) - we may wish to ignore readings in auto (this should be removed eventually), leaving this logic in for now.
		final boolean DISABLE_LIMELIGHT_IN_AUTO = false;
		if (isPoseEstimateAcceptable && !(DriverStation.isAutonomous() && DISABLE_LIMELIGHT_IN_AUTO) && useVisionToggle.get()) {
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
		}

        if (hasTarget() && getCurrentTagID() != -1) {
            int tagId = getCurrentTagID();

            Pose2d scoringPoseLeft = Util4828.calculateRobotPoseFromTagId(
                tagId,
                leftXOffset.get(),   // front-to-tag distance
                leftYOffset.get(),
                Constants.DISTANCE_ROBOT_FRAME_NOBUMPERS_TO_CENTER,             // front-to-center distance
                true            // facing away from tag (adjust as needed)
            );
            if (scoringPoseLeft != null)
                field.getObject("TargetScoringPoseLeft").setPose(scoringPoseLeft);

            Pose2d scoringPoseRight = Util4828.calculateRobotPoseFromTagId(
                tagId,
                rightXOffset.get(),   // front-to-tag distance
                rightYOffset.get(),
                Constants.DISTANCE_ROBOT_FRAME_NOBUMPERS_TO_CENTER,             // front-to-center distance
                true            // facing away from tag (adjust as needed)
            );
            if (scoringPoseRight != null)
                field.getObject("TargetScoringPoseRight").setPose(scoringPoseRight);  
        } else {
            // Clear the previous visualization if no tag is seen
            //field.getObject("TargetScoringPoseLeft").setPose(new Pose2d(0, 0, new Rotation2d(0)));
            //field.getObject("TargetScoringPoseRight").setPose(new Pose2d(0, 0, new Rotation2d(0)));
        }

        // Log information to dashboard
        if (poseEstimate != null)
            field.getObject("LL").setPose(poseEstimate.pose);
        
        // Also log MT1 pose for testing purposes
        PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LIMELIGHT_NAME);
        if (mt1Estimate != null) {
            field.getObject("MegaTag1").setPose(mt1Estimate.pose);
        }
        SmartDashboard.putString("MegaTag1 Pose", mt1Estimate ==  null ? "NULL" : Util4828.formatPose(mt1Estimate.pose));
        
        SmartDashboard.putNumber("LL Tag ID", getCurrentTagID());
        SmartDashboard.putBoolean("LL Has Target", hasTarget());
        SmartDashboard.putBoolean("LL Is Pose Acceptable", isPoseEstimateAcceptable);
        SmartDashboard.putBoolean("LL Publishing Vision", isPoseEstimateAcceptable && useVisionToggle.get());
        SmartDashboard.putString("LL Pose (MT2)", poseEstimate == null ? "NULL" : Util4828.formatPose(poseEstimate.pose));
        SmartDashboard.putNumber("LL Timestamp", poseEstimate == null ? -1 : poseEstimate.timestampSeconds);
    }
}
