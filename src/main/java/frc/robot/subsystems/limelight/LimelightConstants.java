package frc.robot.subsystems.limelight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final String LIMELIGHT_URL = "http://0.0.0.0:5800/stream.mjpg";

    public static final AprilTagFields APRIL_TAG_FIELD_TYPE = AprilTagFields.k2025ReefscapeAndyMark;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(APRIL_TAG_FIELD_TYPE);

    /* =================== */
    /* AUTOALIGN CONSTANTS */
    /* =================== */

    // TODO check the offsets
    // TODO make it so that these values can be changed while robot is enabled through shuffleboard
    // distance from the center of the robot gyro to the edge of the bumpers
    public static final double ROBOT_OFFSET_METERS = Units.inchesToMeters(14);
    // distance from center of the april tag to the reef pole on the right side
    public static final double REEF_RIGHT_OFFSET_PATHFINDER = Units.inchesToMeters(12.5); // 6.47 in
    // distance from center of the april tag to the reef pole on the left side
    public static final double REEF_LEFT_OFFSET_PATHFINDER = Units.inchesToMeters(-2.7);

    public static final double REEF_RIGHT_OFFSET_PID = Units.inchesToMeters(13);
    public static final double REEF_LEFT_OFFSET_PID = Units.inchesToMeters(-4);
    public static final double ROBOT_ROTATION = Math.PI;

    // max velocity in meters per second
    public static final double AUTOALIGN_MAX_VELOCITY = 1.5;
    // max acceleration in meters per second squared
    public static final double AUTOALIGN_MAX_ACCELERATION = 11.5; // max is 11.5
    // max angular velocity in radians per second
    public static final double AUTOALIGN_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540); 
    // max angular acceleration in radians per second squared
    public static final double AUTOALIGN_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(720); // max is 2056

    public static boolean isAligned = false;
    public static boolean inScoringDistance = false;
    public static final double AUTOALIGN_MIN_DISTANCE = -0.1; // meters
    public static final double AUTOALIGN_MAX_DISTANCE = 0.8; // meters
}
