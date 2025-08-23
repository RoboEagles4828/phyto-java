// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
   //SmartDashboard.putNumber("Speed", m_moduleMechanisms[0].getVelocity);
   SmartDashboard.putNumber("Module 0", m_robotContainer.drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());
   SmartDashboard.putNumber("Module 1", m_robotContainer.drivetrain.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());
   SmartDashboard.putNumber("Module 2", m_robotContainer.drivetrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble());
   SmartDashboard.putNumber("Module 3", m_robotContainer.drivetrain.getModule(3).getDriveMotor().getVelocity().getValueAsDouble());

   //SmartDashboard.putNumber("Module0 Angle", m_robotContainer.drivetrain.getModule(0).getSteerMotor().getPosition().getValueAsDouble());

    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
