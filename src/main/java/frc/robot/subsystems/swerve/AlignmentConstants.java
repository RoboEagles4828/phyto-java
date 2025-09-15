package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;

/**
 * Swerve drive specific constants not from the CTRE generator, but added by us.
 */
class AlignmentConstants {
    /**
     * Subdivide the reef faces into four zones by X distance from blue alliance
     * wall. This is to make the target selection process more efficient by
     * eliminating three quarters of the possible targets by a simple pose X value
     * check.
     */
    static final int REEF_FACE_ZONE_COUNT = 4;

    /* The following values are taken from the official field diagrams. */
    /** Alliance wall to nearest reef face. */
    static final double WALL_TO_REEF_INCHES = 144.0;
    /** Depth of the reef zone around the reef (includes line). */
    static final double REEF_ZONE_DEPTH_INCHES = 14.0;
    /** Width of total reef zone (includes lines). */
    static final double REEF_ZONE_WIDTH_INCHES = 93.5;

    /* The following values are calculated from the above. */
    /** Distancr from the alliance wall to the center of there reef (in meters). */
    static final double WALL_TO_REEF_CENTER_X_METERS = Inches
            .of(WALL_TO_REEF_INCHES - REEF_ZONE_DEPTH_INCHES + (REEF_ZONE_WIDTH_INCHES / 2.0)).in(Meters);

    /* The following are constants for autopilot. */
    /**
     * The {@link APConstraints} for autopilot start (acceleration) and end (jerk)
     * behavior.
     */
    static final APConstraints AP_CONSTRAINTS = new APConstraints()
            .withAcceleration(5.0) /* TODO probably needs tuning */
            .withJerk(2.0);

    /** The {@link APProfile} for autopilot tolerances and beeline radius. */
    static final APProfile AP_PROFILE = new APProfile(AP_CONSTRAINTS)
            .withErrorXY(Centimeters.of(2)) /* TODO probably needs tuning */
            .withErrorTheta(Degrees.of(0.5))
            .withBeelineRadius(Centimeters.of(8));

    /** The {@link Autopilot} instance for moving to targets. */
    static final Autopilot AUTO_PILOT = new Autopilot(AP_PROFILE);
}
