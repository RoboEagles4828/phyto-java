package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.PIDSwerve;

public class LimelightConstants {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final String LIMELIGHT_URL = "http://0.0.0.0:5800/stream.mjpg";

    /* =================== */
    /* AUTOALIGN CONSTANTS */
    /* =================== */

    // TODO make it so that these values can be changed while robot is enabled through shuffleboard
    /** The distance from the center of the robot to the edge of the bumpers in m. */
    public static final double ROBOT_OFFSET_METERS = Units.inchesToMeters(16);

    /** The distance from center of the april tag to the reef pole on the right side in m.
     *  This offset is used in the code that autoaligns using pathplanner pathfinder.
    */
    public static final double REEF_RIGHT_OFFSET_PATHFINDER = Units.inchesToMeters(12.5); // 6.47 in
    /** The distance from center of the april tag to the reef pole on the left side in m.
     *  This offset is used in the code that autoaligns using pathplanner pathfinder.
     */
    public static final double REEF_LEFT_OFFSET_PATHFINDER = Units.inchesToMeters(-2.7);

    /** The distance from center of the april tag to the reef pole on the right side in m.
     *  This offset is used in the code that autoaligns using {@link PIDSwerve}.
     */
    public static final double REEF_RIGHT_OFFSET_PID = Units.inchesToMeters(12.7);
    /** The distance from center of the april tag to the reef pole on the left side in m.
     *  This offset is used in the code that autoaligns using {@link PIDSwerve}.
     */
    public static final double REEF_LEFT_OFFSET_PID = Units.inchesToMeters(-3.5);
    /** The transform necessary to align to the right. */
    public static final Transform2d REEF_RIGHT_OFFSET_TRANSFORM = 
        new Transform2d(
            LimelightConstants.ROBOT_OFFSET_METERS,
            LimelightConstants.REEF_RIGHT_OFFSET_PID,
            new Rotation2d(LimelightConstants.ROBOT_ROTATION));
    /** The transform necessary to align to the left. */
    public static final Transform2d REEF_LEFT_OFFSET_TRANSFORM = 
        new Transform2d(
            LimelightConstants.ROBOT_OFFSET_METERS,
            LimelightConstants.REEF_LEFT_OFFSET_PATHFINDER, 
            new Rotation2d(LimelightConstants.ROBOT_ROTATION)
        );

    /** Orientation of the robot relative to the position/orientation of the tag. */
    public static final double ROBOT_ROTATION = Math.PI;

    /** The maximum velocity of the robot while it autoaligns in m/s. */
    public static final double AUTOALIGN_MAX_VELOCITY = .75;
    /** The maximum acceleration of the robot while it autoaligns in m/s^2. */
    public static final double AUTOALIGN_MAX_ACCELERATION = 11.5; // max is 11.5
    /** The maximum angular velocity of the robot while it autoaligns in rad/s. */
    public static final double AUTOALIGN_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540); 
    /** The maximum angular acceleration of the robot while it autoaligns in rad/s^2. */
    public static final double AUTOALIGN_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(720); // max is 2056

    public static boolean isAligned = false;
    public static boolean inScoringDistance = false;
    public static final double AUTOALIGN_MIN_DISTANCE = -0.1; // meters
    public static final double AUTOALIGN_MAX_DISTANCE = 0.1; // meters
    public static final double PID_AUTO_ALIGN_TIMEOUT = 4.0; // seconds
}
