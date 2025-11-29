package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class Util4828 {
    public static String formatPose(Pose2d pose) {
        return String.format(
            "x: %.3f  y: %.3f  rot: %.3f°",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        );
    }

}
