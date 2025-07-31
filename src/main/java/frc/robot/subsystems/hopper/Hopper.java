// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.game.CoralState;

/**
 * The hopper actively funnels coral into the coral cannon during intake operations.
 */
public class Hopper extends SubsystemBase {
  /** Motor used for intake and agitation. */
  private final TalonSRX feedWheels = new TalonSRX(RioBusCANIds.HOPPER_MOTOR_CANID);

  /**
   * Creates the hopper subsystem, configures the motor, and creates game piece state bindings.
   */
  public Hopper() {
    this.feedWheels.setInverted(false);
    // The hopper is usually stopped.
    this.setDefaultCommand(this.stop());
    // Start intaking and remain intaking as long as in the coral intake state.
    CoralState.INTAKE.getTrigger().whileTrue(this.intake());
    // Start agitating and remain agitating as long as in the coral intake state.
    CoralState.HOPPER_JAMMED.getTrigger().whileTrue(this.agitate());
  }

  /**
   * @return a new command to stop the hopper feed wheels.
   */
  private Command stop() {
    // Not runOnce to keep the command running (this is the default).
    // Not run to avoid uneeded CAN bus traffic.
    return this.startEnd(
        () -> this.feedWheels.set(TalonSRXControlMode.PercentOutput, 0.0),
        () -> this.feedWheels.set(TalonSRXControlMode.PercentOutput, 0.0));
  }

  /**
   * @return a new command to run the hopper feed wheels for coral intake.
   */
  private Command intake() {
    return this.run(() -> this.feedWheels.set(TalonSRXControlMode.PercentOutput, HopperConstants.INTAKE_DUTY_CYCLE));
  }

  /**
   * @return a new command to run the hopper feed wheels for coral agitation.
   */
  private Command agitate() {
    return this.run(() -> this.feedWheels.set(TalonSRXControlMode.PercentOutput, HopperConstants.AGITATION_DUTY_CYCLE));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hopper / Stator Current", this.feedWheels.getStatorCurrent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
