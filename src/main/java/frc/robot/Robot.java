// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.game.CoralLevel;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;

public class Robot extends TimedRobot {
    /** The robot subsystems, and trigger and command bindings. */
    private final RobotContainer robotContainer;

    /** Command selected for execution during autonomous mode */
    private Command autonomousCommand = null;

    /**
     * True if autonomous mode ran. If it did, we don't want to seed the initial pose at the start of teleop, as it will
     * have already been set by PathPlanner. Additionally we want to keep the elevator up after autonomous ends in case
     * a piece missed and is stuck under the elevator (bringing the elevator down on a stuck piece can damage the
     * robot).
     */
    private boolean didAutonomousRun = false;

    public Robot() {
        // Instantiate our RobotContainer. Adds button bindings and creates auto chooser.
        // You probably shouldn't put anything else in here. Use robotInit instead.
        this.robotContainer = new RobotContainer();
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Starts the USB camera output
        CameraServer.startAutomaticCapture();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     * 
     * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block every tick in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        SmartDashboard.putString("Coral State", CoralState.getCurrentState().toString());
        SmartDashboard.putString("Elevated Level", ElevatedLevel.TRACKER.getCurrentLevel().toString());        
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // Enabled into autonomous or practice match.
        didAutonomousRun = true;

        // Autonomous always starts with coral loaded. Setting here to cover repeated testing cases.
        CoralState.setCurrentState(CoralState.CARRY);
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        // We are moving to teleop from autonomous (like in a real match)
        if (didAutonomousRun) {
            if (autonomousCommand != null) {
                // Cancel any command still running from autonomous.
                autonomousCommand.cancel();

                // Keep the elevator raised at the end of autonomous.
                ElevatedLevel.TRACKER.setCurrentLevel(CoralLevel.L2);
                CoralState.setCurrentState(CoralState.PREPARE_TO_SCORE);
            }
            else {
                DriverStation.reportError("Auto command is null, but didAutoRun is true. This shouldn't happen!", false);
            }
        } 
        // We are enabling directly into teleop
        else {
            // TODO (ben) seedFieldCentric
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
