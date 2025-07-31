// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static final).
 * Do not put anything functional in this class.
 */
public final class Constants {
  /**
   * Constants related to operator interaction devices.
   */
  public static class OperatorConstants {
    /** Primary drive controller USB port. */
    public static final int DRIVER_CONTROLLER_PORT = 0;
    /** Secondary operator controller USB port. */
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  /**
   * The native roboRIO CAN bus ids. They are managed here to help keep them unique.
   */
  public static class RioBusCANIds {
    public static final int HOPPER_MOTOR_CANID = 13;
    public static final int CANNON_LEFT_MOTOR_CANID = 14;
    public static final int CANNON_RIGHT_MOTOR_CANID = 15;
    public static final int ELEVATOR_LEFT_MOTOR_CANID = 21;
    public static final int ELEVATOR_RIGHT_MOTOR_CANID = 22;
    public static final int ALGAE_MANIPULATOR_WHEEL_MOTOR_CANID = 41;
    public static final int ALGAE_MANIPULATOR_PIVOT_MOTOR_CANID = 42;
  }

  /**
   * The canivore CAN bus ids. They are managed here to help keep them unique.
   */
  public static class CanivoreCANIds {

  }

  /**
   * The roboRIO Digital IO ids. They are managed here to help keep them unique.
   */
  public static class DIOIds {
    public static final int ELEVATOR_BOTTOM_LIMIT_DIO = 9;
    public static final int ELEVATOR_TOP_LIMIT_DIO = 0;
    public static final int ELEVATOR_QUAD_ENCODER_A_CHANNEL = 1;
    public static final int ELEVATOR_QUAD_ENCODER_B_CHANNEL = 2;
  }
}
