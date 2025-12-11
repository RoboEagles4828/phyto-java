// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.game.CoralLevel;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.cannon.Cannon;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.limelight.PIDAutoAlign;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.PathPlannerAutoAlign;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.Util4828;

public class RobotContainer {
	private final Field2d field = new Field2d();

	/* === SUBSYSTEMS === */
	/** The CTRE swerve drivetrain controls the wheels which drive the chassis. */
	private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(field);

	/** The hopper funnels coral to the coral cannon. */
	@SuppressWarnings("unused")
	private final Hopper coralHopper = new Hopper();

	/** The coral cannon deposits coral onto the reef for scoring. */
	@SuppressWarnings("unused")
	private final Cannon coralCannon = new Cannon();

	/** The algae manipulator is used to remove algae from the reef and score them in the barge. */
	private final AlgaeManipulator algaeManipulator = new AlgaeManipulator();

	/** The elevator is used to move game piece manipulators (AlgaeManipulator and Cannon) to various heights. */
	private final Elevator elevator = new Elevator();

	/** The limelight camera used for vision, both pose-estimation and auto-alignment to the reef. */
	private final Limelight limelight = new Limelight(drivetrain, field);

	/* === CTRE SWERVE === */
	// TODO these constants should be in swerve constants folder, not in RobotContainer
	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(TunerConstants.MaxSpeed * 0.1) // Add a 10% deadband
		.withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
		.withDeadband(TunerConstants.MaxAlignmentSpeed * 0.1) // Add a 10% deadband
		.withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	/** Logs swerve data through SignalLogger for sysID  */
	private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

	/* === CONTROLLERS === */
	/** Controller used primarily for driving the robot around the field. */
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

	/** Controller used primarily for operator game piece manipulation. */
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
	
	/* === COMMANDS === */
	// Make the driver controller rumble for 0.5s
	private final Command rumbleDriverControllerCommand = Commands.runOnce(() -> this.driverController.setRumble(RumbleType.kBothRumble, 1.0))
		.andThen(Commands.waitSeconds(0.5))
		.andThen(Commands.runOnce(() -> this.driverController.setRumble(RumbleType.kBothRumble, 0.0)));

	// Set the target elevator height to a given level. Note that this does not immediately move the elevator.
	private final Command setElevatorL1Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L1));
	private final Command setElevatorL2Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L2));
	private final Command setElevatorL3Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L3));
	private final Command setElevatorL4Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L4));

	// Perform coral intake (manual enable/disable of hopper for auto mode).
	// TODO(Ben) - this is kinda $HACKY$ but it should work (maybe); the coral state stuff doesn't seem to be fully thought out.
	private final Command startHopperCommand = new InstantCommand(() -> CoralState.setCurrentState(CoralState.INTAKE));
	private final Command stopHopperCommand = new InstantCommand(() -> endIntakeProcessing());

	// Perform coral intake whle this button is held (for teleop mode).
	private final Command intakeCoralWhileHeldCommand = Commands.startEnd(
			() -> CoralState.setCurrentState(CoralState.INTAKE),
			this::endIntakeProcessing)
			.andThen(rumbleDriverControllerCommand);

	// Attempt to score a Coral onto the Reef (fire a Coral out of the Shooter).
	private final Command scoreCoralCommand = new InstantCommand(() -> { CoralState.setCurrentState(CoralState.SCORE);});

	// Attempt to perform auto-align with PathPlanner for movement.
	private final Command autoAlignLeftPathPlannerCommand = new PathPlannerAutoAlign(limelight, drivetrain, PathPlannerAutoAlign.Side.LEFT).withTimeout(5.0);
	private final Command autoAlignRightPathPlannerCommand = new PathPlannerAutoAlign(limelight, drivetrain, PathPlannerAutoAlign.Side.RIGHT).withTimeout(5.0);

	/* === MEMBER VARIABLES === */
	// Chooser widget which will contain all autonomous routines from PathPlanner and displays on dashboard.
	private final SendableChooser<Command> autoChooser;


	public RobotContainer() {
		// Register commands with PathPlanner so they may be used in autonomous routines.
		NamedCommands.registerCommand("ScoreCoral", scoreCoralCommand);
		NamedCommands.registerCommand("ElevatorL1", setElevatorL1Command);
		NamedCommands.registerCommand("ElevatorL2", setElevatorL2Command);
		NamedCommands.registerCommand("ElevatorL3", setElevatorL3Command);
		NamedCommands.registerCommand("ElevatorL4", setElevatorL4Command);
		NamedCommands.registerCommand("RaiseElevator", elevator.getMoveToAndHoldCommand());
		NamedCommands.registerCommand("AutoAlignLeft", autoAlignLeftPathPlannerCommand);
		NamedCommands.registerCommand("AutoAlignRight", autoAlignRightPathPlannerCommand);
		NamedCommands.registerCommand("StartHopper", startHopperCommand);
		NamedCommands.registerCommand("StopHopper", stopHopperCommand);

		// Create and populate a SendableChooser with the autonomous routines from PathPlanner, and add it to dashboard.
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		SmartDashboard.putData("Field", field);

		Util4828.publishAprilTags(field);

		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		// Note that according to WPILib convention,
		// X is defined as forward
		// Y is defined as to the left
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			// Drive forward with negative Y (left joystick forward)
			// Drive left with negative X (left joystick left)
			// Drive counterclockwise with negative X (right joystick left)
			drivetrain.applyRequest(() -> drive
				.withVelocityX(-driverController.getLeftY() * TunerConstants.MaxSpeed)
				.withVelocityY(-driverController.getLeftX() * TunerConstants.MaxSpeed)
				.withRotationalRate(-driverController.getRightX() * TunerConstants.MaxAngularRate)
			)
		);

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
			drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on left bumper press
		driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		// auto align to the reef
		// TODO make these bindings just povRight/Left without anding with a
		// TODO consider adding new buttons on the driver controller to accomodate auto align
		driverController.b().onTrue(new PIDAutoAlign(drivetrain, driveRR, limelight, driverController, true, true));
		driverController.x().onTrue(new PIDAutoAlign(drivetrain, driveRR, limelight, driverController, false, true));

		operatorController.start().onTrue(Commands.runOnce(() -> SignalLogger.start()));
		operatorController.back().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

		drivetrain.registerTelemetry(logger::telemeterize);

		// Alternative driving scheme for slow robot-relative driving meant for aligning manually
		driverController.a().whileTrue( 
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(-driverController.getLeftY() * TunerConstants.MaxAlignmentSpeed * 0.5)
				.withVelocityY(-driverController.getLeftX() * TunerConstants.MaxAlignmentSpeed * 0.5)
				.withRotationalRate(-driverController.getRightX() * TunerConstants.MaxAlignmentSpeed * 0.5)
			)
		);			
			
		/* Testing basic robot movement in the cardinal directions */
		// Drive straight forward slowly
		driverController.povUp().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(0.1 * TunerConstants.MaxSpeed)
				.withVelocityY(0.0)
				.withRotationalRate(0.0)));
		// Drive straight backward slowly
		driverController.povDown().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(-0.1 * TunerConstants.MaxSpeed)
				.withVelocityY(0.0)
				.withRotationalRate(0.0)));
		// Drive straight right slowly
		driverController.povRight().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(0.0)
				.withVelocityY(-0.1 * TunerConstants.MaxSpeed)
				.withRotationalRate(0)));
		// Drive straight left slowly
		driverController.povLeft().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(0.0)
				.withVelocityY(0.1 * TunerConstants.MaxSpeed)
				.withRotationalRate(0)));

		// Intake button binding. Rumble only happens on normal (not interrupted by button release) completion.
		driverController.leftTrigger().whileTrue(intakeCoralWhileHeldCommand);

		// Driver coral jammed in hopper agitation bindings.
		// On press, change to the hopper jammed state. On release, change to empty to be ready to intake again.
		driverController.rightBumper()
			.whileTrue(Commands.startEnd(
				() -> CoralState.setCurrentState(CoralState.HOPPER_JAMMED),
				() -> CoralState.setCurrentState(CoralState.EMPTY)));

		// Driver prepare to score binding.
		driverController.rightTrigger().onTrue(
			Commands.runOnce(() -> CoralState.setCurrentState(CoralState.PREPARE_TO_SCORE)));
		driverController.rightTrigger().onFalse(
			Commands.runOnce(this::setPostScoreState));

		// Subsystem derived prepare to score to ready to score bindings.
		// TODO when have drive train, add it to this compound trigger.
		final Trigger robotReadyToScoreTrigger = 
			elevator.getReadyToScoreTrigger()
			.and(algaeManipulator.getReadyToScoreTrigger());
		// If preparing to score and subsystems are ready, we are now ready to score.
		CoralState.PREPARE_TO_SCORE.getTrigger().and(robotReadyToScoreTrigger)
				.onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.READY_TO_SCORE)));
		// If ready to score and a subsystem is no longer ready, we are back to preparing to score.
		CoralState.READY_TO_SCORE.getTrigger().and(robotReadyToScoreTrigger.negate())
				.onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.PREPARE_TO_SCORE)));

		// Driver score (coral or algae) binding.
		// Note that the driver should treat the left bumper like a while held in all cases.
		driverController.leftBumper().whileTrue(scoreCoralCommand);

		// Both operator binding for return to carry and elevator to zero (was or'ed with driver pov down).
		// TODO consider going to EMPTY and when we get to zero, run intake for a moment to decide between
		// EMPTY/CARRY.
		operatorController.povDown()
			.onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.CARRY)));

		// Operator target coral scoring level selection bindings.
		operatorController.a().onTrue(setElevatorL1Command);
		operatorController.b().onTrue(setElevatorL2Command);
		operatorController.x().onTrue(setElevatorL3Command);
		operatorController.y().onTrue(setElevatorL4Command);

		// Operator bindings for elevator nudges.
		operatorController.rightTrigger().whileTrue(elevator.nudgeUpCommand());
		operatorController.leftTrigger().whileTrue(elevator.nudgeDownCommand());
		
		// Autoalign triggers
		operatorController.leftBumper().onTrue(autoAlignLeftPathPlannerCommand);
		operatorController.rightBumper().onTrue(autoAlignRightPathPlannerCommand);
	}

	/**
	 * If the coral state is still set to "INTAKE" when called, go to the empty state. This is designed for the intake button
	 * release. If the intake was successful, the state will be carry when we get here and this method will not change
	 * it. If the intake was unsuccessful, we failed to pick up a coral and thus the state is set to "EMPTY".
	 */
	private void endIntakeProcessing() {
		if (CoralState.INTAKE.isCurrent()) {
			CoralState.setCurrentState(CoralState.EMPTY);
		}
	}

	/**
	 * Checks to see if the current task is to dealgae the reef. If so, the current state is set to
	 * {@link CoralState#MAY_HAVE_ALGAE}, otherwise it is set empty. This is designed to only be called on release of
	 * the score button binding.
	 */
	private void setPostScoreState() {
		if (ElevatedLevel.TRACKER.isCurrentAlgaeLevel() && algaeManipulator.isDealgae()) {
			CoralState.setCurrentState(CoralState.MAY_HAVE_ALGAE);
		} else {
			CoralState.setCurrentState(CoralState.EMPTY);
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}