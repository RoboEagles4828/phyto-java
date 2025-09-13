// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Factory methods for simple autos that do not use path following.
 * 
 * <p>
 * TODO when we have a drive train, add just move, and move and score L1 autos.
 */
public final class SimpleAutos {
    /**
     * @return a command that does nothing.
     */
    public static Command doNothing() {
        return Commands.print("Do nothing auto.");
    }

    /**
     * @param drivetrain
     * @param drive
     * @return a command that moves the robot at velocity 1 m/s backward for 3 seconds
     */
    public static Command move(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
        return drivetrain.applyRequest(() -> drive.withVelocityX(-1)
                                                .withVelocityY(0)
                                                .withRotationalRate(0)).withTimeout(3);
    }

    private SimpleAutos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
