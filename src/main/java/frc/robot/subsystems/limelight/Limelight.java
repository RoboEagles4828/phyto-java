package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Limelight extends SubsystemBase {
    
    private CommandSwerveDrivetrain drivetrain; //< reference to the robot's drivetrain

    private PoseEstimate poseEstimate = null; //< most recent pose estimate from limelight; null if no estimate can be made.
    private boolean isPoseEstimateAcceptable = false; //< if the current poseEstimate should be used.

    public Limelight(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
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

    @Override
    public void periodic() {
        // feed the robot's current rotation to the limelight (required for MegaTag2 algorithm)
        LimelightHelpers.SetRobotOrientation(LimelightConstants.LIMELIGHT_NAME, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
        // get the latest pose estimate from the drivetrain and check its quality
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LIMELIGHT_NAME);
        isPoseEstimateAcceptable = verifyPoseEstimate(poseEstimate);

        // feed vision estimate to the drivetrain, if the reading is good.
        // todo(ben) - we may wish to ignore readings in auto (this should be removed eventually), leaving this logic in for now.
		final boolean DISABLE_LIMELIGHT_IN_AUTO = false;
		if (isPoseEstimateAcceptable && !(DriverStation.isAutonomous() && DISABLE_LIMELIGHT_IN_AUTO)){
			drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
		}

        SmartDashboard.putNumber("LL Tag ID", getCurrentTagID());
        SmartDashboard.putBoolean("LL Has Target", hasTarget());
        SmartDashboard.putBoolean("LL Is Pose Estimate Acceptable", isPoseEstimateAcceptable);
        SmartDashboard.putString("LL Pose Estimate", poseEstimate == null ? "NULL" : poseEstimate.pose.toString());
        SmartDashboard.putNumber("LL Pose Timestamp", poseEstimate == null ? -1 : poseEstimate.timestampSeconds);
    }
}
