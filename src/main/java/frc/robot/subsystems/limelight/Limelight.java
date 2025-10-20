package frc.robot.subsystems.limelight;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String limelightName;
    private IntSupplier currentTagReading;
    private BooleanSupplier hasTarget;
    private BooleanSupplier acceptLimelightReading;
    private HttpCamera cameraFeed;

    private PoseEstimate mt1;

    public Limelight() {
        limelightName = LimelightConstants.LIMELIGHT_NAME;
        
        currentTagReading = () -> (int) LimelightHelpers.getFiducialID(limelightName);
        hasTarget = () -> LimelightHelpers.getTV(limelightName);
        acceptLimelightReading = this::acceptLimelightUpdate;
        cameraFeed = new HttpCamera("Limelight", LimelightConstants.LIMELIGHT_URL);
        this.addCamera();

    }

    /**
     * Add limelight camera feed to SmaartDashboard
     */
    public void addCamera() {
        CameraServer.addCamera(cameraFeed);
        Shuffleboard.getTab("SmartDashboard").add(cameraFeed);
    }

    /**
     * Perform Filtering on camera reading.
     */
    public boolean acceptLimelightUpdate() {
        if (mt1.tagCount != 1 || mt1.rawFiducials.length != 1) {
            return false;
        } else {
            // If ambiguity is too high, reject
            if (mt1.rawFiducials[0].ambiguity > .7) {
                return false;
            }
            // If we're too far from the tag, reject
            if (mt1.rawFiducials[0].distToCamera > 3) {
                return false;
            }
        }
        return true;
    }

    public PoseEstimate getLimeLightPoseEstimate() {
        if (acceptLimelightReading.getAsBoolean()) {
            return mt1;
        } else {
            return null;
        }
    }

    public int getCurrentTagID() {
        return currentTagReading.getAsInt();
    }

    public boolean hasTarget() {
        return hasTarget.getAsBoolean();
    }

    public boolean getAcceptLimelightReading() {
        return acceptLimelightReading.getAsBoolean();
    }

    @Override
    public void periodic() {
        // Print limelight info to SmartDashboard
        mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LIMELIGHT_NAME);
        SmartDashboard.putNumber("Limelight Fiducial ID", getCurrentTagID());
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putString("Limelight Pose", mt1.pose.toString());
        SmartDashboard.putNumber("Limelight Timestamp", mt1.timestampSeconds);
        SmartDashboard.putBoolean("Limelight Update Accepted", getAcceptLimelightReading());
    }
}
