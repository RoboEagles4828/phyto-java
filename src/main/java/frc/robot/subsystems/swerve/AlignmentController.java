package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevelTracker;

/**
 * An instance of this class (created and owned by {@link CommandSwerveDrivetrain}) uses the autopilot command from
 * {@link #alignCommand(Supplier)} to move the robot to a specified target (see {@link APTarget}).
 * 
 * <p>
 * The target and be explicitly set or calculated based of the current robot position and intended game task (see
 * {@link ElevatedLevelTracker}). Also see {@link #setTarget(APTarget)} for details on target lifecycle.
 */
public class AlignmentController {
    private final AprilTagFieldLayout fieldLayout;
    private final CommandSwerveDrivetrain swerve;
    private final Command alignmentCommand;
    private APTarget target = null;

    /**
     * Field centric (with facing angle) swerve request used during autopilot
     * motion.
     * 
     * <p>
     * If I am reading the documentation correctly (not mid-page:
     * https://therekrab.github.io/autopilot/prerequisites.html), the coordinate
     * system for autopilot motion is always NWU with (0,0) at the far right of the
     * blue alliance wall. Leave the forward perspective value as it is.
     * 
     * TODO verify.
     */
    private final SwerveRequest.FieldCentricFacingAngle m_fieldRelativeRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(4, 0, 0); /* TODO probably needs tuning */

    /** Used to quickly narrow the reef face options during target calculation from 12 to just 3. */
    private final List<List<ReefFaceTargets>> reefFaceTagZones = new ArrayList<>(AlignmentConstants.REEF_FACE_ZONE_COUNT);

    /**
     * Creates the pieces required to run autopilot, handle target lifecycle, and populates the reef tag zones, which
     * are used to facilitate target calculation.
     * 
     * @param swerve
     *            the swerve drive train for this robot.
     */
    AlignmentController(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        this.alignmentCommand = this.alignCommand(this::targetSupplier);
        CoralState.PREPARE_TO_SCORE.getTrigger().onFalse(Commands.runOnce(this::discardTarget));
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        reefFaceTagZones.set(0, List.of(
                new ReefFaceTargets(17),
                new ReefFaceTargets(18),
                new ReefFaceTargets(19))); // Closest to blue wall
        reefFaceTagZones.set(1, List.of(
                new ReefFaceTargets(20),
                new ReefFaceTargets(21),
                new ReefFaceTargets(22))); // Back of blue reef
        reefFaceTagZones.set(2, List.of(
                new ReefFaceTargets(9),
                new ReefFaceTargets(10),
                new ReefFaceTargets(11))); // Back of red reef
        reefFaceTagZones.set(3, List.of(
                new ReefFaceTargets(6),
                new ReefFaceTargets(7),
                new ReefFaceTargets(8))); // Closest to red wall
    }

    /**
     * Used to set an explicit target. This should be used for every target during
     * auto and in response to some drive team gestures during teleop.
     * 
     * <p>
     * Note that after the alignment command is complete, the
     * {@link AlignmentController} forgets about the target and sets up to calculate
     * the next one. However, a new target can be set at anytime before the next
     * alignment.
     * 
     * @param target the new target for alignment.
     */
    public void setTarget(final APTarget target) {
        this.target = target;
    }

    /**
     * A convenience method for autos and other command groups to be able to set a
     * target and align to it in one shot.
     * 
     * @param target the new target for alignment.
     */
    public void setTargetAndAlign(final APTarget target) {
        this.setTarget(target);
        this.align();
    }

    /**
     * Set the target to null (call upon alignment completion). This sets the
     * controller up to calculate the next target, unless another explicit target is
     * set.
     */
    private void discardTarget() {
        this.target = null;
    }

    /**
     * Run the alignment command.
     */
    public void align() {
        this.alignmentCommand.schedule();
    }

    /**
     * @return true if the alignment command is running.
     */
    public boolean isAligning() {
        return this.alignmentCommand.isScheduled();
    }

    /**
     * The target supplier for the
     * {@link CommandSwerveDrivetrain#align(java.util.function.Supplier)} command.
     * 
     * @return the current target for autopilot movement.
     */
    private APTarget targetSupplier() {
        if (this.target == null) {
            final Pose2d currentPose = this.swerve.getCurrentPose();
            Pose2d targetPose = currentPose; // Stay put if we can not calculate target

            // TODO if carry and target level is coral calculate target based on position on the field.
            final int reefFaceTagZone = getReefFaceTagZone(currentPose.getTranslation().getMeasureX());
            // TODO if target level is dealgae calculate target based on position on the field.
            // TODO if target level is score barge and in zone 1 and left of center
            // TODO if empty and in zone 0, closest loading station

            setTarget(new APTarget(targetPose));
        }
        return this.target;
    }

    /**
     * Returns a commmand that uses autopilot to move the robot to the supplied
     * {@link APTarget}.
     * 
     * <p>
     * Note that only one of these should be needed to move the robot to any target
     * during a match. The value returned from the supplier should change as needed.
     * 
     * @param targetSupplier a supplier of the target to close to.
     * @return a new autopilot command.
     */
    private Command alignCommand(final Supplier<APTarget> targetSupplier) {
        Command alignCmd = this.swerve.startRun(
                () -> targetSupplier.get(), // Not required but ensures target present before first run
                () -> {
                    final ChassisSpeeds robotRelativeSpeeds = this.swerve.getRobotRelativeSpeeds();
                    final Pose2d pose = this.swerve.getCurrentPose();

                    final APResult output = AlignmentConstants.AUTO_PILOT.calculate(pose, robotRelativeSpeeds, targetSupplier.get());

                    /* these speeds are field relative */
                    final double veloX = output.vx().magnitude();
                    final double veloY = output.vy().magnitude();
                    final Rotation2d headingReference = output.targetAngle();

                    this.swerve.setControl(this.m_fieldRelativeRequest
                            .withVelocityX(veloX)
                            .withVelocityY(veloY)
                            .withTargetDirection(headingReference));
                })
                .until(() -> AlignmentConstants.AUTO_PILOT.atTarget(this.swerve.getCurrentPose(), targetSupplier.get()))
                .finallyDo(this.swerve::stop);
        return alignCmd;
    }

    /**
     * Narrows the list of possible reef faces from 12 to 3 by simply considering the robot's current position relative
     * to the blue alliance wall.
     * 
     * @param x
     *            the current X position of the robot
     * @return the zone index into the {@link #reefFaceTagZones} list.
     */
    private int getReefFaceTagZone(final Distance x) {
        double xMeters = x.baseUnitMagnitude();
        if (xMeters < AlignmentConstants.WALL_TO_REEF_CENTER_X_METERS) {
            return 0;
        } else if (xMeters < (this.fieldLayout.getFieldLength() / 2.0)) {
            return 1;
        } else if (xMeters < (this.fieldLayout.getFieldLength() - AlignmentConstants.WALL_TO_REEF_CENTER_X_METERS)) {
            return 2;
        }
        return 3;
    }

    /**
     * An instance of this class provides the interesting target poses for a reef face by april tag id.
     */
    private class ReefFaceTargets {
        final int aprilTagId;
        final Pose3d aprilTagPose;
        final Pose2d leftReefPose;
        final Pose2d rightReefPose;
        final Pose2d dealgaePose;

        ReefFaceTargets(final int aprilTagId) {
            this.aprilTagId = aprilTagId;
            this.aprilTagPose = AlignmentController.this.fieldLayout.getTagPose(aprilTagId).get();
            final Pose2d aprilTagPose2d = this.aprilTagPose.toPose2d();
            // TODO calculate these three with along face and to robot center offsets
            this.leftReefPose = aprilTagPose2d;
            this.rightReefPose = aprilTagPose2d;
            this.dealgaePose = aprilTagPose2d;
        }
    }
}
