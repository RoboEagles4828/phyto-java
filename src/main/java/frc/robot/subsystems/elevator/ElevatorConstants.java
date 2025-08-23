package frc.robot.subsystems.elevator;

import java.util.Map;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import frc.robot.game.AlgaeLevel;
import frc.robot.game.CoralLevel;
import frc.robot.game.ElevatedLevel;

/**
 * Defines constants used only within the elevator subsystem. Note that the python code defined motion magic constants
 * but we did not use motion magic, so they are not present here. Also, the python code defined current limit constants,
 * but they were never applied to the motor configuration, so they are not present here.
 */
class ElevatorConstants {
    /**
     * Immutable map of {@link ElevatedLevel} to elevator positions in mechanism rotations from 0. Note that this map
     * does not support null keys nor values.
     */
    static final Map<ElevatedLevel, Double> LEVEL_TO_POSITION = Map.of(
            CoralLevel.L1, 1.105,
            CoralLevel.L2, 1.9,
            CoralLevel.L3, 2.84,
            CoralLevel.L4, 4.102,
            AlgaeLevel.DEALGAE_LOW, 2.0,
            AlgaeLevel.DEALGAE_HIGH, 3.0,
            AlgaeLevel.SCORE_BARGE, 4.102);

    /** PID slot 0 is usually used. Use slot 1 to go above this threshold. */
    static final double PID_SLOT_POSITION_THRESHOLD = 3.5;

    /**
     * Defines the PID constants for moving the elevator to a level using {@link PositionVoltage} control.
     * 
     * <p>
     * Movement to zero and nudges are done using {@link DutyCycleOut} control.
     */
    static class PID_CONFIG {
        static final double GRAVITY = 0.0;
        static final double STATIC = 0.0;
        static final double VELOCITY = 0.0;
        static final double ACCELERATION = 0.0;
        static final double INTEGRAL = 0.0;
        static final double DERIVATIVE = 0.0;
        /** kP for L4 in slot 1. */
        static final double PROPORTIONAL_L4 = 7.5;
        /** kP for other levels in slot 0. */
        static final double PROPORTIONAL_OTHERS = 9.0;
    }

    /** PID configuration slot for moving up to L4 */
    static final int L4_SLOT = 1;
    /** PID configuration slot for moving up to other levels */
    static final int OTHERS_SLOT = 0;

    /** Closed loop voltage ramp rate (see {@link ClosedLoopRampsConfigs#VoltageClosedLoopRampPeriod}) */
    static final double CLOSED_LOOP_VOLTAGE_RAMP_SEC = 0.5;
    /** Open loop voltage ramp rate (see {@link OpenLoopRampsConfigs#VoltageOpenLoopRampPeriod}) */
    static final double OPEN_LOOP_VOLTAGE_RAMP_SEC = 0.5;

    /**
     * Gear ratio is set in the {@link FeedbackConfigs#SensorToMechanismRatio} field to transform
     * {@link CoreTalonFX#getPosition()} return values to mechanism rotations.
     */
    static final double GEAR_RATIO = 18.0;
    /**
     * On target tolerance for PID controled movement. Due to the {@link #GEAR_RATIO} usage, this is specified in
     * mechanism rotations. This was 0.07 on the old code, which is about 1.17 inches either way (a bit much). Combine
     * that with how it altered the target on hold and we can see the source of some inaccuracies. This value is tighter
     * and we do not modify the target anymore to hold.
     */
    static final double ON_TARGET_TOLERANCE_MECH_ROTATIONS = 0.03; // About 0.5 inches either way.

    /** Duty cycle for holding at zero (TODO consider small negative value). */
    static final double AT_ZERO_DUTY_CYCLE = 0.0;
    /** Duty cycle for movement to zero. */
    static final double MOVE_TO_ZERO_DUTY_CYCLE = -0.25; // TODO this was -0.7 in old code but that seemed fast.
    /** Duty cycle for nudges up. */
    static final double NUDGE_UP_DUTY_CYCLE = 0.3;
    /** Duty cycle for nudges down. */
    static final double NUDGE_DOWN_DUTY_CYCLE = -0.1;

    /** Time to debounce elevator bottom detection for encoder zeroing. */
    static final double BOTTOM_DETECTION_DEBOUNCE_SEC = 0.1;
}
