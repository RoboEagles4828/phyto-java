package frc.robot.subsystems.limelight;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.util.Util4828;

public class PathPlannerAutoAlign extends Command {
    public enum Side {
        LEFT, RIGHT
    }

    private final Limelight limelight;

    private Command internalCommand = null;
    private Side side;

    public PathPlannerAutoAlign(
            Limelight limelight,
            CommandSwerveDrivetrain drivetrain,
            Side side
    ) {
        this.limelight = limelight;
        this.side = side;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Must have a limelight target
        if (!limelight.hasTarget() || limelight.getCurrentTagID() == -1) {
            System.out.println("[AutoAlign] No target visible, abort");
            internalCommand = Commands.print("AutoAlign failed - no target");
            return;
        }

        int tagId = limelight.getCurrentTagID();

        Pose2d scoringPose = Util4828.calculateRobotPoseFromTagId(
            tagId,
            side == Side.LEFT ? limelight.getLeftXOffset() : limelight.getRightXOffset(),   // front-to-tag distance
            side == Side.LEFT ? limelight.getLeftYOffset() : limelight.getRightYOffset(),
            Constants.DISTANCE_ROBOT_FRAME_BUMPERS_TO_CENTER,             // front-to-center distance
            true            // facing away from tag (adjust as needed)
        );

        if (scoringPose == null) {
            System.out.println("[AutoAlign] Could not compute scoring pose, abort");
            internalCommand = Commands.print("AutoAlign failed - invalid target pose");
            return;
        }

        System.out.println("[AutoAlign] Target pose = " + scoringPose);

        // PathPlanner constraints (tune these!)
        PathConstraints constraints = new PathConstraints(
                0.5,  // max velocity (m/s)
                0.5,  // max accel (m/s^2)
                0.5,  // max angular vel (rad/s)
                5.0   // max angular accel
        );

        // Generate the PP pathfinding command
        internalCommand = AutoBuilder.pathfindToPose(scoringPose, constraints);

        internalCommand.initialize();
    }

    @Override
    public void execute() {
        if (internalCommand != null) {
            internalCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (internalCommand != null) {
            internalCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return internalCommand != null && internalCommand.isFinished();
    }
}
