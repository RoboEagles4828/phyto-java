// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SimpleAutos;
import frc.robot.game.AlgaeLevel;
import frc.robot.game.CoralLevel;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.cannon.Cannon;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // START From CTRE
    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // END from CTRE

    /** The hopper used to funnel coral to the coral cannon. */
    @SuppressWarnings("unused")
    private final Hopper coralHopper = new Hopper();
    /** The coral cannon used for intake and scoring. */
    @SuppressWarnings("unused")
    private final Cannon coralCannon = new Cannon();
    /** The algae manipulator is used to remove algae from the reef and score them in the barge. */
    private final AlgaeManipulator algaeManipulator = new AlgaeManipulator();
    /** The elevator is used to move game piece manipulators between levels. */
    private final Elevator elevator = new Elevator();

    /** Controller used primarily for driving the robot around the field. */
    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);
    /** Controller used primarily for operator game piece manipulation. */
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.OPERATOR_CONTROLLER_PORT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        // START from CTRE
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                // Drive forward with negative Y (forward)
                // Drive left with negative X (left)
                // Drive counterclockwise with negative X (left)
                drivetrain.applyRequest(() -> drive.withVelocityX(driverController.getLeftY() * MaxSpeed)
                        .withVelocityY(driverController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Commentted out for merge. Other things are on these.
        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Drive straight forward slowly
        driverController.povUp().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(0.1 * MaxSpeed)
                        .withVelocityY(0 * MaxSpeed)
                        .withRotationalRate(0 * MaxAngularRate)));
        // Drive straight backward slowly
        driverController.povDown().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(-0.1 * MaxSpeed)
                        .withVelocityY(0 * MaxSpeed)
                        .withRotationalRate(0 * MaxAngularRate)));
        // Drive straight right slowly
        driverController.povRight().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(0 * MaxSpeed)
                        .withVelocityY(-0.1 * MaxSpeed)
                        .withRotationalRate(0 * MaxAngularRate)));
        // Drive straight left slowly
        driverController.povLeft().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(0 * MaxSpeed)
                        .withVelocityY(0.1 * MaxSpeed)
                        .withRotationalRate(0 * MaxAngularRate)));
        // END from CTRE

        // Driver intake control bindings.
        // Intake button binding. Rumble only happens on normal (not interrupted by button release) completion.
        this.driverController.leftTrigger()
                .whileTrue(Commands.startEnd(
                        () -> CoralState.setCurrentState(CoralState.INTAKE),
                        this::endIntakeProcessing)
                        .andThen(Commands.runOnce(() -> this.driverController.setRumble(RumbleType.kBothRumble, 1.0))
                                .andThen(Commands.waitSeconds(0.5))
                                .andThen(Commands
                                        .runOnce(() -> this.driverController.setRumble(RumbleType.kBothRumble, 0.0)))));

        // Driver coral jammed in hopper agitation bindings.
        // On press, change to the hopper jammed state. On release, change to empty to be ready to intake again.
        this.driverController.rightBumper()
                .whileTrue(Commands.startEnd(
                        () -> CoralState.setCurrentState(CoralState.HOPPER_JAMMED),
                        () -> CoralState.setCurrentState(CoralState.EMPTY)));

        // Driver prepare to score binding.
        this.driverController.rightTrigger()
                .onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.PREPARE_TO_SCORE)));

        // Subsystem derived prepare to score to ready to score bindings.
        // TODO when have drive train, add it to this compound trigger.
        final Trigger robotReadyToScoreTrigger = this.elevator.getReadyToScoreTrigger()
                .and(this.algaeManipulator.getReadyToScoreTrigger());
        // If preparing to score and subsystems are ready, we are now ready to score.
        CoralState.PREPARE_TO_SCORE.getTrigger().and(robotReadyToScoreTrigger)
                .onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.READY_TO_SCORE)));
        // If ready to score and a subsystem is no longer ready, we are back to preparing to score.
        CoralState.READY_TO_SCORE.getTrigger().and(robotReadyToScoreTrigger.negate())
                .onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.PREPARE_TO_SCORE)));

        // Driver score (coral or algae) bindings.
        // Note that the driver should treat the left bumper like a while held in all cases.
        // On press, change to score state. For coral, it will change to empty on its own.
        this.driverController.leftBumper().onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.SCORE)));
        // For removing algae from the reef, we want to behave like whileTrue and go to the may have algae state. The
        // driver can observe if algae is actually present and decide what to do next.
        this.driverController.leftBumper().negate()
                .and(ElevatedLevel.TRACKER.getIsCurrentAlgaeLevelTrigger())
                .and(() -> this.algaeManipulator.isDealgae())
                .onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.MAY_HAVE_ALGAE)));

        // Driver controller algae scoring level selection bindings.
        this.driverController.a()
                .onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.DEALGAE_LOW)));
        this.driverController.b()
                .onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.DEALGAE_HIGH)));
        this.driverController.y()
                .onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.SCORE_BARGE)));

        // Both operator binding for return to carry and elevator to zero (was or'ed with driver pov down).
        // TODO consider going to EMPTY and when we get to zero, run intake for a moment to decide between
        // EMPTY/CARRY.
        this.operatorController.povDown()
                .onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.CARRY)));

        // Operator target coral scoring level selection bindings.
        this.operatorController.a()
                .onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L1)));
        this.operatorController.x()
                .onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L2)));
        this.operatorController.b()
                .onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L3)));
        this.operatorController.y()
                .onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L4)));

        // Operator bindings for elevator nudges.
        this.operatorController.rightTrigger().whileTrue(this.elevator.nudgeUpCommand());
        this.operatorController.leftTrigger().whileTrue(this.elevator.nudgeDownCommand());

        // Operator bindings for manual algae manipulator arm movement.
        this.operatorController.povLeft().whileTrue(this.algaeManipulator.manualDeployArm());
        this.operatorController.povRight().whileTrue(this.algaeManipulator.manualRetractArm());
        // Operator bindings for manual algae manipulator wheel movement.
        this.operatorController.leftBumper().whileTrue(this.algaeManipulator.manualRemoveAlgaeFromReef());
        this.operatorController.rightBumper().whileTrue(this.algaeManipulator.manualScoreAlgaeIntoBarge());
    }

    /**
     * If the coral state is still intaking when called, go to the empty state. This is designed for the intake button
     * release. If the intake was successful, the state will be carry when we get here and this method will not change
     * it.
     */
    private void endIntakeProcessing() {
        if (CoralState.INTAKE.isCurrent()) {
            CoralState.setCurrentState(CoralState.EMPTY);
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return SimpleAutos.doNothing();
    }
}
