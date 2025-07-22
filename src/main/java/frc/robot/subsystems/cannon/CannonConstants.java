package frc.robot.subsystems.cannon;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * Constants unique to the cannon and only needed inside the cannon package.
 */
class CannonConstants {
    /** When activated, motor current is limited to this value. */
    final static double CURRENT_LIMIT_AMPS = 40.0;
    /** Current threshold to activate current limit. */
    final static double CURRENT_THRESHOLD_AMPS = 60.0;
    /** Time threshold must be exceeded to activate current limit. */
    final static double CURRENT_THRESHOLD_SEC = 3.0;

    final static SupplyCurrentLimitConfiguration SUPPLY_CONFIG = new SupplyCurrentLimitConfiguration(true,
            CURRENT_LIMIT_AMPS, CURRENT_THRESHOLD_AMPS, CURRENT_THRESHOLD_SEC);

    /** Percent power while intaking. */
    final static double INTAKE_DUTY_CYCLE = 0.3;
    /** Stall debounce time for coral detection. */
    final static double INTAKE_STALL_DEBOUNCE_SEC = 0.3;
    /** Current draw over this limit is considered a stall. */
    final static double INTAKE_STALL_THRESHOLD_AMPS = 10.0;

    /** Left side duty cycle for straight shot. */
    final static double LEFT_SCORE_STRAIGHT_DUTY_CYCLE = 0.6;
    /** Right side duty cycle for straight shot. */
    final static double RIGHT_SCORE_STRAIGHT_DUTY_CYCLE = 0.6;

    /** Left side duty cycle for L1 twisted shot. */
    final static double LEFT_SCORE_L1_DUTY_CYCLE = 0.65;
    /** Right side duty cycle for L1 twisted shot. */
    final static double RIGHT_SCORE_L1_DUTY_CYCLE = 0.0;
}
