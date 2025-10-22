package frc.robot.subsystems.limelight;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;



public class AutoAlign extends SequentialCommandGroup {

    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    
    private final boolean targetRight;

    private Transform3d tagToTarget;

    public AutoAlign(
        CommandSwerveDrivetrain drivetrain,
        Limelight limelight,
        boolean targetRight
    ){
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        
        this.targetRight = targetRight;

        if (targetRight) {
            tagToTarget = new Transform3d(
                new Translation3d(
                    LimelightConstants.ROBOT_OFFSET_METERS,
                    LimelightConstants.REEF_RIGHT_OFFSET,
                    0
                ),
                new Rotation3d(0, 0, LimelightConstants.ROBOT_ROTATION)
            );
        } else {
            tagToTarget = new Transform3d(
                new Translation3d(
                    LimelightConstants.ROBOT_OFFSET_METERS,
                    LimelightConstants.REEF_LEFT_OFFSET,
                    0
                ),
                new Rotation3d(0, 0, LimelightConstants.ROBOT_ROTATION)
            );
        }

        addRequirements(drivetrain, limelight);

        final var idle = new SwerveRequest.Idle();
        addCommands(
            new InstantCommand(() -> drivetrain.applyRequest(() -> idle)),
            new WaitCommand(0.20),
            new DeferredCommand(() -> autoAlignCommand(), Set.of(drivetrain, limelight)),
            new InstantCommand(() -> drivetrain.applyRequest(() -> idle))
        );
    }


    public  Command autoAlignCommand(){        
        if (limelight.hasTarget() && limelight.getAcceptLimelightReading()) {
            int tagID = limelight.getCurrentTagID();

            Pose3d aprilTagPos = LimelightConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get();
            Pose2d targetPos = aprilTagPos.transformBy(tagToTarget).toPose2d();

            return AutoBuilder.pathfindToPose(
                targetPos,
                new PathConstraints(
                    LimelightConstants.AUTOALIGN_MAX_VELOCITY,
                    LimelightConstants.AUTOALIGN_MAX_ACCELERATION,
                    LimelightConstants.AUTOALIGN_MAX_ANGULAR_VELOCITY,
                    LimelightConstants.AUTOALIGN_MAX_ANGULAR_ACCELERATION
                )
            );
        } else {
            return new InstantCommand(
                () -> System.out.println("Conditions not met for autoalign; No acceptable target detected by Limelight."));
        }
    } 
}

