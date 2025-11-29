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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.PIDSwerve;
import frc.robot.subsystems.swerve.PIDSwerve;



public class AutoAlign extends SequentialCommandGroup {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric driveRR;
    private final Limelight limelight;
    private final CommandXboxController driverController;
    
    private final boolean targetRight;
    private final boolean usePID;

    private Transform3d tagToTarget;

    public AutoAlign(
        CommandSwerveDrivetrain drivetrain,
        SwerveRequest.RobotCentric driveRR,
        Limelight limelight,
        CommandXboxController driverController,
        boolean targetRight,
        boolean usePID
    ){
        this.drivetrain = drivetrain;
        this.driveRR = driveRR;
        this.limelight = limelight;
        this.driverController = driverController;
        
        this.targetRight = targetRight;
        this.usePID = usePID;

        if (targetRight) {
            tagToTarget = new Transform3d(
                new Translation3d(
                    LimelightConstants.ROBOT_OFFSET_METERS,
                    LimelightConstants.REEF_RIGHT_OFFSET_PATHFINDER,
                    0
                ),
                new Rotation3d(0, 0, LimelightConstants.ROBOT_ROTATION)
            );
        } else {
            tagToTarget = new Transform3d(
                new Translation3d(
                    LimelightConstants.ROBOT_OFFSET_METERS,
                    LimelightConstants.REEF_LEFT_OFFSET_PATHFINDER,
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
            new DeferredCommand(() -> autoAlignCommand(), Set.of(drivetrain, limelight)).until(
                () -> LimelightConstants.isAligned).withTimeout(LimelightConstants.PID_AUTO_ALIGN_TIMEOUT),
            new InstantCommand(() -> drivetrain.applyRequest(() -> idle)),
            new WaitCommand(0.50),
            new InstantCommand(() -> this.driverController.setRumble(RumbleType.kBothRumble, 0.0))
        );
    }


    public Command autoAlignCommand(){        
        if (limelight.hasTarget() && limelight.isPoseEstimateAcceptable()) {
            int tagID = limelight.getCurrentTagID();

            Pose3d aprilTagPos = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get();
            Pose2d targetPos = aprilTagPos.transformBy(tagToTarget).toPose2d();
            
            if (!usePID){
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
                return new PIDSwerve(drivetrain, limelight, !targetRight);
            }
            
        } else {
            return new InstantCommand(
                () -> driverController.setRumble(RumbleType.kBothRumble, 1.0));
        }
    }
}

