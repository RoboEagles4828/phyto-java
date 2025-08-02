// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /** Command selected for execution during {@link #autonomousInit()} */
  private Command autonomousCommand;

  /** The robot subsystems, and trigger and command bindings. */
  private final RobotContainer robotContainer;

  /** Used to make repeated teleop testing more efficient by setting the coral state appropriately. */
  private boolean enabledDirectToTeleOp = true;

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    this.robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putString("Coral State", CoralState.getCurrentState().toString());
    SmartDashboard.putString("Elevated Level", ElevatedLevel.getCurrentLevel().toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Enabled into autonomous or practice match.
    this.enabledDirectToTeleOp = false;
    // Autonomous always starts with coral loaded. Setting here to cover repeated testing cases.
    CoralState.setCurrentState(CoralState.CARRY);
    this.autonomousCommand = this.robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (this.autonomousCommand != null) {
      this.autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (this.autonomousCommand != null) {
      this.autonomousCommand.cancel();
    }

    // During testing, when we enable directly to teleop, the robot does not normally have a coral.
    if (this.enabledDirectToTeleOp) {
      CoralState.setCurrentState(CoralState.EMPTY);
    }
  }

  /**
   * Overridden to reset for direct to teleop enablement. When auto initializes, it will set it to false.
   */
  @Override
  public void teleopExit() {
    this.enabledDirectToTeleOp = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
