
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.game.CoralLevel;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightConstants;

public class PIDSwerve extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private boolean isScoringLeft;
    private double offset = 0.0;

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

    private double mult;
    private Pose2d currentPose;
    private Pose2d reefScoringPose;
    private Pose2d targetPose;
    /** The distance from the target position. */
    private double distance;
    private double rotationError;

    private boolean canAlign = true;

    public PIDSwerve(
        CommandSwerveDrivetrain drivetrain,
        Limelight limelight,
        boolean isScoringLeft
    ) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        
        translationController = new ProfiledPIDController(5, 0, 0, new Constraints(
            LimelightConstants.AUTOALIGN_MAX_VELOCITY, LimelightConstants.AUTOALIGN_MAX_ACCELERATION));
        translationController.setTolerance(Units.inchesToMeters(0.125));

        rotationController = new ProfiledPIDController(5, 0, 0, new Constraints(
            LimelightConstants.AUTOALIGN_MAX_ANGULAR_VELOCITY, LimelightConstants.AUTOALIGN_MAX_ANGULAR_ACCELERATION));
        rotationController.setTolerance(Units.degreesToRadians(0.50));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        this.isScoringLeft = isScoringLeft;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        DriverStation.getAlliance().ifPresent((alliance) -> {
            mult = (alliance == Alliance.Red) ? -1.0 : 1.0;
        });

        translationController.setConstraints(new Constraints(
            LimelightConstants.AUTOALIGN_MAX_VELOCITY, LimelightConstants.AUTOALIGN_MAX_ACCELERATION));

        offset = getScoringPositionOffset(isScoringLeft);

        int tagID = limelight.getCurrentTagID();
        reefScoringPose = LimelightConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d()
            .plus(new Transform2d(LimelightConstants.ROBOT_OFFSET_METERS, offset, new Rotation2d(LimelightConstants.ROBOT_ROTATION)));
        
        currentPose = drivetrain.getState().Pose;

        SmartDashboard.putString("target pos", reefScoringPose.toString());
        SmartDashboard.putString("current pos", currentPose.toString());

        double velocity = mult * projection(
            new Translation2d(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond
            ),
            reefScoringPose.getTranslation().minus(currentPose.getTranslation()));

        distance = currentPose.getTranslation().getDistance(reefScoringPose.getTranslation());
        translationController.reset(distance, velocity);

        rotationError = currentPose.getRotation().getRadians() - reefScoringPose.getRotation().getRadians();
        rotationController.reset(MathUtil.angleModulus(currentPose.getRotation().getRadians()), drivetrain.getState().Speeds.omegaRadiansPerSecond);

        canAlign = (
            currentPose.getTranslation().getDistance(reefScoringPose.getTranslation()) >= 0.5 || 
            CoralState.getCurrentState() == CoralState.PREPARE_TO_SCORE);
        // if(canAlign) {
            targetPose = reefScoringPose;
        // }
        // else if(!canAlign) {
        //     targetPose = reefScoringPose.plus(new Transform2d(-0.55, offset, new Rotation2d(0)));
        // }
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getState().Pose;
        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        translationController.reset(distance, translationController.getSetpoint().velocity);
        rotationError = MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());

        double rotationPIDOutput = rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()),
            targetPose.getRotation().getRadians());
        double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;
        
        double scalar =  scalar(distance);
        double drivePIDOutput = translationController.calculate(distance, 0);
        double driveSpeed = mult * scalar * translationController.getSetpoint().velocity + drivePIDOutput;
        Rotation2d direction = new Rotation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

        
        drivetrain.setControl(drive
            .withVelocityX(driveSpeed * direction.getCos())
            .withVelocityY(driveSpeed * direction.getSin())
            .withRotationalRate(omega));
        
        if(currentPose.getTranslation().getDistance(reefScoringPose.getTranslation()) >= 0.5 && !canAlign) {
            targetPose = reefScoringPose;
            canAlign = true;
        }

        LimelightConstants.isAligned = finishedAligning(); // && canAlign;
        LimelightConstants.inScoringDistance = inScoringDistance(); // && canAlign;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    public double getScoringPositionOffset(boolean isScoringLeft) {
        return (isScoringLeft) ? LimelightConstants.REEF_LEFT_OFFSET_PID :  LimelightConstants.REEF_RIGHT_OFFSET_PID;
    }

    public boolean finishedAligning() {
        return (distance < Units.inchesToMeters(1.5)) && (Math.abs(rotationError) < Units.degreesToRadians(2.5));
    }

    public boolean inScoringDistance() {
        return (distance < 0.3);
    }

    private double projection(Translation2d v1, Translation2d onto){
        Vector<N2> velocity = VecBuilder.fill(v1.getX(), v1.getY());
        Vector<N2> translation = VecBuilder.fill(onto.getX(), onto.getY());
        Vector<N2> projection = velocity.projection(translation);
        if(projection.dot(translation) > 0) {
              return -Math.sqrt(projection.dot(projection));  
        } else {
           return Math.sqrt(projection.dot(projection));     
        }
    }

    private double scalar(double distance){
        if(distance > LimelightConstants.AUTOALIGN_MAX_DISTANCE){
            return 1.0;
        } else if (LimelightConstants.AUTOALIGN_MIN_DISTANCE < distance && distance < LimelightConstants.AUTOALIGN_MAX_DISTANCE){
            return MathUtil.clamp((1 / (LimelightConstants.AUTOALIGN_MAX_DISTANCE - LimelightConstants.AUTOALIGN_MIN_DISTANCE)) * (distance - LimelightConstants.AUTOALIGN_MIN_DISTANCE), 0, 1);
        } else {
            return 0.0;
        }
    }
}