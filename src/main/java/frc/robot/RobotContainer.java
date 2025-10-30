// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.SimpleAutos;
import frc.robot.game.AlgaeLevel;
import frc.robot.game.CoralLevel;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.cannon.Cannon;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.limelight.AutoAlign;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private final SendableChooser<Command> autoChooser;
	/* ========== */
	/* SUBSYSTEMS */
	/* ========== */

	/** The CTRE swerve drivetrain used to move the chassis. */
	private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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

	/** The limelight camera used for vision and autoalign. */
	private final Limelight limelight = new Limelight();
	
	/** Command to score coral */
	private final Command setElevatorL1Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L1));
	private final Command setElevatorL2Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L2));
	private final Command setElevatorL3Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L3));
	private final Command setElevatorL4Command = new InstantCommand(() -> ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L4));

	private final Command scoreCoralCommand = new InstantCommand(() -> {
		//if (elevator.isMovingToAndHoldingLevel()) {
		CoralState.setCurrentState(CoralState.SCORE);
		//}
	});

	/* ======================= */
	/* CTRE SWERVE NECESSITIES */
	/* ======================= */
	
	// TODO these constants should be in swerve constants folder, not in RobotContainer
	// kSpeedAt12Volts desired top speed in m/s
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);	
	// 3/4 of a rotation per second max angular velocity in rad/s (=42.97183 deg/s)
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
	// max speed for manual alignment
	private double MaxAlignmentSpeed = (MetersPerSecond.of(1)).in(MetersPerSecond);

	// Setting elevator levels for various scoring/intake positions

	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(MaxSpeed * 0.1) // Add a 10% deadband
		.withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
		.withDeadband(MaxAlignmentSpeed * 0.1) // Add a 10% deadband
		.withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	/** Logs swerve data through SignalLogger for sysID  */
	private final Telemetry logger = new Telemetry(MaxSpeed);


	/* =========== */
	/* CONTROLLERS */
	/* =========== */
			
	/** Controller used primarily for driving the robot around the field. */
	private final CommandXboxController driverController = new CommandXboxController(
			OperatorConstants.DRIVER_CONTROLLER_PORT);

	/** Controller used primarily for operator game piece manipulation. */
	private final CommandXboxController operatorController = new CommandXboxController(
			OperatorConstants.OPERATOR_CONTROLLER_PORT);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		NamedCommands.registerCommand("ScoreCoral", scoreCoralCommand);
		NamedCommands.registerCommand("ElevatorL1", setElevatorL1Command);
		NamedCommands.registerCommand("ElevatorL2", setElevatorL2Command);
		NamedCommands.registerCommand("ElevatorL3", setElevatorL3Command);
		NamedCommands.registerCommand("ElevatorL4", setElevatorL4Command);
		NamedCommands.registerCommand("RaiseElevator", elevator.getMoveToAndHoldCommand());
		
		autoChooser = AutoBuilder.buildAutoChooser();
		// Add simple autos to chooser.
		this.autoChooser.addOption("Do Nothing", SimpleAutos.doNothing());
		this.autoChooser.setDefaultOption("Move Off Line", SimpleAutos.move(drivetrain, driveRR));
		SmartDashboard.putData("Auto Chooser", autoChooser);

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
		// Note that according to WPILib convention,
		// X is defined as forward
		// Y is defined as to the left
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			// Drive forward with negative Y (left joystick forward)
			// Drive left with negative X (left joystick left)
			// Drive counterclockwise with negative X (right joystick left)
			drivetrain.applyRequest(() -> drive
				.withVelocityX(-driverController.getLeftY() * MaxSpeed)
				.withVelocityY(-driverController.getLeftX() * MaxSpeed)
				.withRotationalRate(-driverController.getRightX() * MaxAngularRate)
			)
		);

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
			drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		/* Commented out for merge as there are other methods binded to these buttons.
		driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
		driverController.b().whileTrue(drivetrain.applyRequest(() ->
			point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), driverController.getLeftX()))));
		*/

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
		driverController.b().onTrue(new AutoAlign(drivetrain, limelight, true));
		driverController.x().onTrue(new AutoAlign(drivetrain, limelight, false));

		operatorController.start().onTrue(Commands.runOnce(() -> SignalLogger.start()));
		operatorController.back().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

		drivetrain.registerTelemetry(logger::telemeterize);

		// Alternative driving scheme for slow robot-relative driving meant for aligning manually
		// TODO test that it drives slowly when a is held
		// TODO test if the repeat command needs to be added for it to work
		driverController.a().whileTrue( // new RepeatCommand(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(-driverController.getLeftY() * MaxAlignmentSpeed * 0.5)
				.withVelocityY(-driverController.getLeftX() * MaxAlignmentSpeed * 0.5)
				.withRotationalRate(-driverController.getRightX() * MaxAlignmentSpeed * 0.5)
			) // )
		);			
			
		/* Testing basic robot movement in the cardinal directions */
		// Drive straight forward slowly
		driverController.povUp().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(0.1 * MaxSpeed)
				.withVelocityY(0.0)
				.withRotationalRate(0.0)));
		// Drive straight backward slowly
		driverController.povDown().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(-0.1 * MaxSpeed)
				.withVelocityY(0.0)
				.withRotationalRate(0.0)));
		// Drive straight right slowly
		driverController.povRight().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(0.0)
				.withVelocityY(-0.1 * MaxSpeed)
				.withRotationalRate(0)));
		// Drive straight left slowly
		driverController.povLeft().whileTrue(
			drivetrain.applyRequest(() -> driveRR
				.withVelocityX(0.0)
				.withVelocityY(0.1 * MaxSpeed)
				.withRotationalRate(0)));

		// Intake button binding. Rumble only happens on normal (not interrupted by button release) completion.
		driverController.leftTrigger().whileTrue(
			Commands.startEnd(
				() -> CoralState.setCurrentState(CoralState.INTAKE),
				this::endIntakeProcessing
			)
			.andThen(
				Commands.runOnce(() -> this.driverController.setRumble(RumbleType.kBothRumble, 1.0))
				.andThen(Commands.waitSeconds(0.5))
				.andThen(Commands.runOnce(() -> this.driverController.setRumble(RumbleType.kBothRumble, 0.0))))
		);

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

		// Driver controller algae scoring level selection bindings.
		// driverController.b()
		// 	.onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.DEALGAE_LOW)));
		// driverController.x()
		// 	.onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.DEALGAE_HIGH)));
		// driverController.y()
		// 	.onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.SCORE_BARGE)));
		operatorController.povRight()
			.onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.DEALGAE_LOW)));
		operatorController.povLeft()
			.onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.DEALGAE_HIGH)));
		operatorController.povUp()
			.onTrue(Commands.runOnce(() -> ElevatedLevel.TRACKER.setCurrentLevel(AlgaeLevel.SCORE_BARGE)));

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

		// Operator bindings for manual algae manipulator arm movement.
		// operatorController.povLeft().whileTrue(algaeManipulator.manualDeployArm());
		// operatorController.povRight().whileTrue(algaeManipulator.manualRetractArm());

		// Operator bindings for manual algae manipulator wheel movement.
		operatorController.leftBumper().whileTrue(algaeManipulator.manualRemoveAlgaeFromReef());
		operatorController.rightBumper().whileTrue(algaeManipulator.manualScoreAlgaeIntoBarge());
		
		// Operator binding to reset elevator encoder.
		// operatorController.back().onTrue(elevator.resetElevatorEncoder());
	}

	/**
	 * The commands V3 framework is coming in 2027. They will be adding the ability to scope triggers to modes (auto,
	 * teleop, etc) and/or a command (trigger only active while the command is running). For now, we have to scope them
	 * ourselves, but is not to hard. Isolating the auto only triggers here for ease of maintanence.
	 * 
	 * <p>
	 * What follows is how I think you could use this to get the path auto to score, first just once. And then, maybe
	 * more in the furture.
	 * 
	 * <p>
	 * This method is not currently called anywhere. If you call if from robot container construction, I believe, to get
	 * the now working path auto to try to score, you would modify the auto defintion to put the path in a sequential
	 * group (the root sequence) with the path first and then a named command that just sets the current coral state to
	 * prepare to score. When prepare to score completes (elevator is on target), the state will transistion to ready to
	 * score and the trigger defined here will fire.
	 * 
	 * <p>
	 * For multi-coral auto, you add to the root sequence a named commmand that waits until the current state is no
	 * longer SCORE. That is followed, in the same root sequence, by a parallel group to a path to the human player
	 * station and a named command to lower the elevator. Then, a named command to transition coral state to intake.
	 * Then, a named command that waits until coral state is CARRY. These last two could be combined by defining a
	 * single named command using startEnd (I think).
	 * 
	 * <p>
	 * Next is the path to the second score followed by transition to prepare to score. Rinse and repeat.
	 */
	private void configureAutoBindings() {
		final Trigger autoScoreTrigger = RobotModeTriggers.autonomous().and(CoralState.READY_TO_SCORE.getTrigger());
		autoScoreTrigger.onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.SCORE)));
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

    public void displayPoseEstimate() {
		SmartDashboard.putString("Drivetrain Pose Estimate: ", drivetrain.getState().Pose.toString()); 
    }

	public void addVisionMeasurement() {
        PoseEstimate mt1 = limelight.getLimeLightPoseEstimate();
		if (mt1 != null){
			drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
		}
    }
}