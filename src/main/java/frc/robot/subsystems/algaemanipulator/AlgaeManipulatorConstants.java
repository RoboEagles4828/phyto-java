package frc.robot.subsystems.algaemanipulator;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * Constants unique to the algae manipulator and only needed inside the algae manipulator package.
 */
public class AlgaeManipulatorConstants {
    /** When activated, motor current is limited to this value. */
    final static double CURRENT_LIMIT_AMPS = 30.0;
    /** Current threshold to activate current limit. */
    final static double CURRENT_THRESHOLD_AMPS = 50.0;
    /** Time threshold must be exceeded to activate current limit. */
    final static double CURRENT_THRESHOLD_SEC = 3.0;

    final static SupplyCurrentLimitConfiguration SUPPLY_CONFIG = new SupplyCurrentLimitConfiguration(true,
            CURRENT_LIMIT_AMPS, CURRENT_THRESHOLD_AMPS, CURRENT_THRESHOLD_SEC);

    /** Stall debounce time for pivot arm end of range detection. */
    final static double PIVOT_STALL_DEBOUNCE_SEC = 0.3;
    /** Current draw over this limit is considered a pivot stall. */
    final static double PIVOT_STALL_THRESHOLD_AMPS = 20.0; // TODO verify this.
    /** Only wait this long for arm deploy or retract pivot stall. */
    final static double PIVOT_STALL_TIMEOUT_SEC = 2.0; // TODO find proper value.

    /** Percent power while deploying the arm to remove algae from the reef. */
    final static double DEPLOY_ARM_DUTY_CYCLE = 0.2; // TODO verify sign and magnitude
    /** Percent power while retracting the arm. */
    final static double RETRACT_ARM_DUTY_CYCLE = -0.2; // TODO verify sign and magnitude

    final static double INTAKE_DUTY_CYCLE = 0.6; // TODO verify sign and magnitude
    final static double SCORE_BARGE_DUTY_CYCLE = -0.6; // TODO verify sign and magnitude
    final static double SCORE_BARGE_DURATION_SEC = 1.75;
}
