package frc.robot.commands;

import java.util.NoSuchElementException;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;



public class AutoAlign extends SequentialCommandGroup {
    AprilTagFieldLayout aprilTagFieldLayout;
    
    CommandSwerveDrivetrain drivetrain;
    
    boolean targetRight;

    Transform3d tagToGoal;
    Transform3d tagToGoalFinal;

    public AutoAlign(
        CommandSwerveDrivetrain drivetrain,
        boolean targetRight
    ){
        this.drivetrain = drivetrain;
        this.targetRight = targetRight;
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AutoAlignConstants.APRIL_TAG_FIELD_TYPE);

        tagToGoal = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(40),
                0,
                0
            ),
            new Rotation3d(0, 0, Math.PI)
        );

        if (targetRight) {
            tagToGoalFinal = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(15),
                    Units.inchesToMeters(6.47),
                    0
                ),
                new Rotation3d(0, 0, Math.PI)
            );
        } 
        else {
            tagToGoalFinal = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(15),
                    Units.inchesToMeters(-6.47),
                    0
                ),
                new Rotation3d(0, 0, Math.PI)
            );
        }

        addRequirements(drivetrain);

        final var idle = new SwerveRequest.Idle();
        addCommands(
            new InstantCommand(() -> drivetrain.applyRequest(() -> idle)),
            new WaitCommand(0.33),
            new DeferredCommand(() -> autoAlignCommand(), Set.of(drivetrain)),
            new InstantCommand(() -> drivetrain.applyRequest(() -> idle))
        );
    }


    public  Command autoAlignCommand(){        
        int tagID = (int) LimelightHelpers.getFiducialID(AutoAlignConstants.LIMELIGHT_NAME);
        
        try{
            var aprilTagPose = aprilTagFieldLayout.getTagPose(tagID).get();

            var goalPose = aprilTagPose.transformBy(tagToGoal).toPose2d();
            var goalPoseFinal = aprilTagPose.transformBy(tagToGoalFinal).toPose2d();

            return AutoBuilder.pathfindToPose(
                goalPoseFinal,
                new PathConstraints(
                    3.5,
                    2.5,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                )
            );
        }
        catch(NoSuchElementException e){
            System.out.println("Tag ID " + tagID + " not found in field layout.");
        }       

        return Commands.none();
    }

    
}

