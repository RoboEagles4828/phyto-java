// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SimpleAutos;
import frc.robot.game.CoralState;
import frc.robot.game.ReefLevel;
import frc.robot.subsystems.cannon.Cannon;
import frc.robot.subsystems.hopper.Hopper;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** The hopper used to funnel coral to the coral cannon. */
  @SuppressWarnings("unused")
  private final Hopper coralHopper = new Hopper();
  /** The coral cannon used for intake and scoring. */
  @SuppressWarnings("unused")
  private final Cannon coralCannon = new Cannon();

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
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Driver intake control bindings.
    // Change to the coral intake state on press.
    this.driverController.leftTrigger().onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.INTAKE)));
    // On release of same trigger and still in coral intake state, return to empty.
    // If we successfully completed intake or got jammed, this compound trigger will not fire.
    this.driverController.leftTrigger().negate().and(CoralState.INTAKE.getTrigger())
        .onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.EMPTY)));

    // Driver coral jammed in hopper agitation bindings.
    // On press, change to the coral jammed in hopper state. On release, change to empty to be ready to intake again.
    this.driverController.rightBumper()
        .onTrue(Commands.startEnd(() -> CoralState.setCurrentState(CoralState.HOPPER_JAMMED),
            () -> CoralState.setCurrentState(CoralState.EMPTY)));

    // Driver score coral bindings.
    // On press, change to score coral state. It will change to empty on its own.
    this.driverController.leftBumper().onTrue(Commands.runOnce(() -> CoralState.setCurrentState(CoralState.SCORE)));

    // Operator target reef level selection bindings.
    this.operatorController.a().onTrue(Commands.runOnce(() -> ReefLevel.setCurrentLevel(ReefLevel.L1)));
    this.operatorController.x().onTrue(Commands.runOnce(() -> ReefLevel.setCurrentLevel(ReefLevel.L2)));
    this.operatorController.b().onTrue(Commands.runOnce(() -> ReefLevel.setCurrentLevel(ReefLevel.L3)));
    this.operatorController.y().onTrue(Commands.runOnce(() -> ReefLevel.setCurrentLevel(ReefLevel.L4)));
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
