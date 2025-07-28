package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOIds;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;
import frc.robot.game.ReefLevel;

/**
 * The elevator subsystem controls the movement of the elevator to scoring levels and its return to zero.
 */
public class Elevator extends SubsystemBase {
    /**
     * The next {@link ElevatedLevel} to move the elevator when moving to and holding a scoring level. Note that this
     * must never be null since {@link ElevatorConstants#LEVEL_TO_POSITION} does not support null keys.
     */
    private ElevatedLevel nextMovingToPostionElevatedLevel = ReefLevel.L1;
    /** The current elevator target position in mechanism rotations. */
    private double currentTargetPosition = 0.0;
    /** Usually slot 0, but use slot 1 for highest targets. */
    private int currentTargetPositionPIDSlot = 0;

    /** Right side elevator motor is the leader. */
    private final TalonFX rightMotorLeader = new TalonFX(RioBusCANIds.ELEVATOR_RIGHT_MOTOR_CANID);
    /** Left side elevator motor follows the right side. */
    private final TalonFX leftMotorFollower = new TalonFX(RioBusCANIds.ELEVATOR_LEFT_MOTOR_CANID);

    /** Encoder tracking elevator position. Counts on rise and fall of each channel (4x). */
    private final Encoder elevatorPositionEncoder = new Encoder(
            DIOIds.ELEVATOR_QUAD_ENCODER_A_CHANNEL,
            DIOIds.ELEVATOR_QUAD_ENCODER_B_CHANNEL,
            false,
            EncodingType.k4X);
    /** Detect elevator at bottom to stop and zero the {@link Elevator#elevatorPositionEncoder}. */
    private final DigitalInput bottomLimitSwitch = new DigitalInput(DIOIds.ELEVATOR_BOTTOM_LIMIT_DIO);
    /** Debounce bottom detection to avoid false positives from vibration. */
    private final Debouncer zeroingDebounce = new Debouncer(ElevatorConstants.BOTTOM_DETECTION_DEBOUNCE_SEC);
    /** Detect elevator at top to stop. */
    private final DigitalInput topLimitSwitch = new DigitalInput(DIOIds.ELEVATOR_TOP_LIMIT_DIO);

    /** PID control to any scoring level. Initialize to 0.0 for match start at bottom. */
    private final PositionVoltage toAnyLevel = new PositionVoltage(0.0).withEnableFOC(true);
    /** Duty cycle control for return to 0.0 and nudges. */
    private final DutyCycleOut toZeroOrNudge = new DutyCycleOut(0.0);

    /**
     * Creates the elevator subsystem, configures the motors, and creates game piece state bindings.
     */
    public Elevator() {
        /** Configure left to follow right, but in opposite direction. */
        this.leftMotorFollower.setControl(new Follower(RioBusCANIds.ELEVATOR_RIGHT_MOTOR_CANID, true));

        /** Used to configure motors and PID slots. */
        final TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;
        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorCfg.Slot0.GravityType = motorCfg.Slot1.GravityType = GravityTypeValue.Elevator_Static;
        motorCfg.Slot0.kG = motorCfg.Slot1.kG = ElevatorConstants.PID_CONFIG.GRAVITY;
        motorCfg.Slot0.kS = motorCfg.Slot1.kS = ElevatorConstants.PID_CONFIG.STATIC;
        motorCfg.Slot0.kV = motorCfg.Slot1.kV = ElevatorConstants.PID_CONFIG.VELOCITY;
        motorCfg.Slot0.kA = motorCfg.Slot1.kA = ElevatorConstants.PID_CONFIG.ACCELERATION;
        motorCfg.Slot0.kI = motorCfg.Slot1.kI = ElevatorConstants.PID_CONFIG.INTEGRAL;
        motorCfg.Slot0.kD = motorCfg.Slot1.kD = ElevatorConstants.PID_CONFIG.DERIVATIVE;
        motorCfg.Slot0.kP = ElevatorConstants.PID_CONFIG.PROPORTIONAL_OTHERS;
        motorCfg.Slot1.kP = ElevatorConstants.PID_CONFIG.PROPORTIONAL_L4;
        motorCfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ElevatorConstants.CLOSED_LOOP_VOLTAGE_RAMP_SEC;
        motorCfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = ElevatorConstants.OPEN_LOOP_VOLTAGE_RAMP_SEC;

        this.rightMotorLeader.getConfigurator().apply(motorCfg);
        this.leftMotorFollower.getConfigurator().apply(motorCfg); // TODO is this needed? Doubt it.

        /** We start at zero, and zeroing is common to coral and algae, make it the default. */
        this.setDefaultCommand(gotoAndStopAtZero());
        /** Move to and hold a scoring (or jam) position when appropriate. */
        CoralState.PREPARE_TO_SCORE.getTrigger()
                .or(CoralState.SCORE.getTrigger())
                .or(CoralState.ELEVATOR_JAMMED.getTrigger())
                .whileTrue(this.gotoAndHoldCurrentTargetPosition());
    }

    /**
     * See the notes on {@link #setNextMovingToPostionElevatedLevel(ElevatedLevel)}. Also note that this is the current
     * target if we are now moving and holding.
     * 
     * @return the level to be targeted the next time we move to a scoring position. Never null.
     */
    public ElevatedLevel getNextMovingToPostionElevatedLevel() {
        return this.nextMovingToPostionElevatedLevel;
    }

    /**
     * If we are currently moving to or holding a level (See
     * {@link ElevatorConstants#MOVING_TO_AND_HOLDING_COMMAND_NAME}), this new next level is immediately applied and the
     * elevator will move to it. Otherwise, it is simply stored for the next movement.
     * 
     * @param nextMovingToPostionElevatedLevel
     *            the level to be targeted the next time we move to a scoring position. Note that this level does not
     *            change if this is null.
     */
    public void setNextMovingToPostionElevatedLevel(final ElevatedLevel nextMovingToPostionElevatedLevel) {
        if (nextMovingToPostionElevatedLevel != null) {
            this.nextMovingToPostionElevatedLevel = nextMovingToPostionElevatedLevel;
            if (this.getCurrentCommand().getName().startsWith(ElevatorConstants.MOVING_TO_AND_HOLDING_COMMAND_NAME)) {
                this.setCurrentTargetPosition();
            }
        }
    }

    /**
     * This sets the target position to move or hold to in mechanism target rotations from 0. The proper PID slot for
     * the movement is also selected.
     */
    private void setCurrentTargetPosition() {
        this.currentTargetPosition = ElevatorConstants.LEVEL_TO_POSITION
                .get(this.getNextMovingToPostionElevatedLevel());
        this.currentTargetPositionPIDSlot = 0;
        if (this.currentTargetPosition > ElevatorConstants.PID_SLOT_POSITION_THRESHOLD) {
            this.currentTargetPositionPIDSlot = 1;
        }
    }

    /** The command to move and hold the elevator at a predetermined scoring level. */
    private Command gotoAndHoldCurrentTargetPosition() {
        return this.startRun(
                this::setCurrentTargetPosition,
                this::gotoAndHoldCurrentTargetPositionRun)
                .withName(ElevatorConstants.MOVING_TO_AND_HOLDING_COMMAND_NAME);
    }

    /** For method reference lambda expressions in commands that move and hold position. */
    private void gotoAndHoldCurrentTargetPositionRun() {
        this.rightMotorLeader.setControl(
                this.toAnyLevel.withPosition(this.currentTargetPosition)
                        .withSlot(this.currentTargetPositionPIDSlot)
                        .withLimitForwardMotion(!this.topLimitSwitch.get())
                        .withLimitReverseMotion(this.bottomLimitSwitch.get()));
    }

    /**
     * @return the command to goto and stop at zero. This is the default command.
     */
    private Command gotoAndStopAtZero() {
        return this.run(
                () -> this.rightMotorLeader.setControl(
                        this.toZeroOrNudge.withOutput(ElevatorConstants.MOVE_TO_ZERO_DUTY_CYCLE)
                                .withLimitReverseMotion(this.bottomLimitSwitch.get())))
                .until(this::isAtBottom)
                .andThen(run(() -> this.rightMotorLeader.setControl(
                        this.toZeroOrNudge.withOutput(ElevatorConstants.AT_ZERO_DUTY_CYCLE))));
    }

    /**
     * @return true if the debounced bottom has been detected, or false otherwise.
     */
    private boolean isAtBottom() {
        final boolean atBottom = this.zeroingDebounce.calculate(this.bottomLimitSwitch.get());
        if (atBottom) {
            this.rightMotorLeader.setPosition(0.0);
            this.elevatorPositionEncoder.reset();
        }
        return atBottom;
    }
}
