package frc.robot.subsystems.limelight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    public static final String LIMELIGHT_NAME = "limelight-";
    public static final String LIMELIGHT_URL = "http://limelight.local:5801/stream.mjpg";

    public static final AprilTagFields APRIL_TAG_FIELD_TYPE = AprilTagFields.k2025ReefscapeAndyMark;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(APRIL_TAG_FIELD_TYPE);

    /* =================== */
    /* AUTOALIGN CONSTANTS */
    /* =================== */

    // TODO check the offsets
    // TODO make it so that these values can be changed while robot is enabled through shuffleboard
    // distance from the center of the robot gyro to the edge of the bumpers
    public static final double ROBOT_OFFSET_METERS = Units.inchesToMeters(15);
    // distance from center of the april tag to the reef pole on the right side
    public static final double REEF_RIGHT_OFFSET = Units.inchesToMeters(6.47);
    // distance from center of the april tag to the reef pole on the left side
    public static final double REEF_LEFT_OFFSET = Units.inchesToMeters(-6.47);
    public static final double ROBOT_ROTATION = Math.PI;

    // max velocity in meters per second
    public static final double AUTOALIGN_MAX_VELOCITY = 3.5;
    // max acceleration in meters per second squared
    public static final double AUTOALIGN_MAX_ACCELERATION = 2.5;
    // max angular velocity in radians per second
    public static final double AUTOALIGN_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540);
    // max angular acceleration in radians per second squared
    public static final double AUTOALIGN_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(720);
}
