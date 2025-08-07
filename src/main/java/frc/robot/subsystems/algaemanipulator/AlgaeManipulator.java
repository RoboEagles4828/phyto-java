package frc.robot.subsystems.algaemanipulator;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.game.AlgaeLevel;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;

/**
 * The algae manipulator is used to remove algae from the reef and, if held during the dealgae operation, score the
 * algae into the barge. There are commands to deploy and retract the arm, and to run the dealgae/scoring wheel.
 * 
 * <p>
 * TODO The python code for the manipulator has some serious issues. For example, the code and comments disagree on
 * which duty cycle sign pivots the arm up or down. Going with the code here. Also, the "deAlgaefyingCommand" command
 * defined in the python robot container, appears to timeout much too quickly. We should be detecting stall or a slow
 * down of the wheel here (or perhaps leave to operator via a whileTrue). Finally, if holding an algae, we may want to
 * keep a small amount of power to the wheel to help hold it.
 * 
 * <p>
 * TODO Currently (and this may be ok), this commands here do not change coral state so as to move the elevator to the
 * proper level for algae work. That is left to the button bindings in robot container. This may or may not be correct.
 * This is just a reminder to consider it.
 */
public class AlgaeManipulator extends SubsystemBase {
    /** Motor to dealgae the reef (inward) and score (outward). */
    private final TalonSRX wheel = new TalonSRX(RioBusCANIds.ALGAE_MANIPULATOR_WHEEL_MOTOR_CANID);
    /** Motor to pivot the arm between stowed, dealgae, and carry/score. */
    private final TalonSRX pivot = new TalonSRX(RioBusCANIds.ALGAE_MANIPULATOR_PIVOT_MOTOR_CANID);
    /** Arm pivot stall detection debounce to detect end of motion. */
    private final Debouncer pivotStallDetection = new Debouncer(
            AlgaeManipulatorConstants.PIVOT_STALL_DEBOUNCE_SEC,
            DebounceType.kBoth);
    /** Set true on arm pivot stall or preparing barge shot. Set false on next non-zero duty cycle. */
    private boolean readyToScore = false;
    /** A {@link Trigger} that is true when not scoring algae or the manipulator is properly configured. */
    private final Trigger readyToScoreTrigger = new Trigger(this::evaluateReadyToScore);
    // TODO add manual arm

    /**
     * Creates the algae manipulator subsystem, configures the motors, and creates game piece state bindings.
     */
    public AlgaeManipulator() {
        this.wheel.configSupplyCurrentLimit(AlgaeManipulatorConstants.SUPPLY_CONFIG);
        this.pivot.configSupplyCurrentLimit(AlgaeManipulatorConstants.SUPPLY_CONFIG);

        // The algae manipulator is usually fully stopped.
        this.setDefaultCommand(this.stop());

        // Yes, we are reusing coral triggers.
        // TODO consider refactoring state enums (or elim AlgaeState and rename CoralState).
        ElevatedLevel.TRACKER.getIsCurrentAlgaeLevelTrigger().and(CoralState.PREPARE_TO_SCORE.getTrigger())
                .whileTrue(prepareToScore());
        // Score algae while elevated to an algae scoring position and the coral state is scoring.
        // Yes, we are reusing the same SCORE trigger.
        ElevatedLevel.TRACKER.getIsCurrentAlgaeLevelTrigger().and(CoralState.SCORE.getTrigger())
                .whileTrue(this.score());
        // TODO Handle current elevated level changes. Need this to smoothly go from dealgae to barge loading.
        // ElevatedLevel.TRACKER.getChangeEvent().onChange(this::handleElevatedLevelChange);
    }

    /**
     * This trigger can be combined with similar triggers from other subsystems to decide when to transition from
     * {@link CoralState#PREPARE_TO_SCORE} to {@link CoralState#READY_TO_SCORE} and vice versa.
     * 
     * @return a trigger that is true when not scoring algae, or we are scoring algae and the manipulator is properly
     *         configured.
     */
    public Trigger getReadyToScoreTrigger() {
        return this.readyToScoreTrigger;
    }

    /**
     * @return true when not scoring algae, or we are scoring algae and the manipulator is properly configured.
     */
    private boolean evaluateReadyToScore() {
        return (!ElevatedLevel.TRACKER.isCurrentAlgaeLevel()) || this.readyToScore;
    }

    /**
     * @return a new command that stops the wheel and pivot. It will stay alive, so it is suitable to be the default
     *         command.
     */
    private Command stop() {
        // Not runOnce to keep the command running (this is the default).
        // Not run to avoid uneeded CAN bus traffic.
        return this.startEnd(this::stopAll, this::stopAll);
    }

    /**
     * When preparing to remove algae from the reef, configure arm in preparation for scoring.
     * 
     * <p>
     * When preparing to load algae into the barge, stop and signal ready.
     */
    private Command prepareToScore() {
        return new ConditionalCommand(
                deployArm(),
                stop().andThen(runOnce(() -> this.readyToScore = true)),
                this::isDealgae);
    }

    /**
     * @return true if the current scoring level is for dealgae work on the reef.
     */
    public boolean isDealgae() {
        final ElevatedLevel currentLevel = ElevatedLevel.TRACKER.getCurrentLevel();
        return (currentLevel == AlgaeLevel.DEALGAE_LOW) || (currentLevel == AlgaeLevel.DEALGAE_HIGH);
    }

    /**
     * Remove algae from the reef or score algae into the barge depending on current scoring level.
     * 
     * @return a command to score algae (remove from reef or place in barge).
     */
    private Command score() {
        return new ConditionalCommand(
                removeAlgaeFromReef(),
                scoreAlgaeIntoBarge(),
                this::isDealgae);
    }

    /**
     * Utility method to stop the wheel and pivot.
     */
    private void stopAll() {
        this.stopWheel();
        this.stopPivot();
    }

    /**
     * Utility method to stop the wheel.
     */
    private void stopWheel() {
        this.setWheelDutyCycle(0.0);
    }

    /**
     * Utility method to stop the pivot.
     */
    private void stopPivot() {
        this.setPivotDutyCycle(0.0);
    }

    /**
     * Utility method to set the pivot to a particular duty cycle.
     * 
     * <p>
     * Note that this must be the only place the pivot duty cycle is set.
     * 
     * <p>
     * Also note that a non-zero duty cycle results in {@link #readyToScore} being set to false.
     * 
     * @param dutyCycle
     *            the duty cycle to set on the pivot.
     */
    private void setPivotDutyCycle(final double dutyCycle) {
        this.pivot.set(TalonSRXControlMode.PercentOutput, dutyCycle);
        if (dutyCycle != 0.0) {
            this.readyToScore = false;
        }
    }

    /**
     * Utility method to set the wheel to a particular duty cycle.
     * 
     * <p>
     * Note that this must be the only place the wheel duty cycle is set.
     * 
     * @param dutyCycle
     *            the duty cycle to set on the wheel.
     */
    private void setWheelDutyCycle(final double dutyCycle) {
        this.wheel.set(TalonSRXControlMode.PercentOutput, dutyCycle);
    }

    /**
     * Returns a command to deploy the arm (move down or outward) via the pivot in order to remove algae from the reef.
     * 
     * @return a command to deploy the algae arm.
     */
    private Command deployArm() {
        return this
                .startEnd(
                        () -> this.setPivotDutyCycle(AlgaeManipulatorConstants.DEPLOY_ARM_DUTY_CYCLE),
                        this::stopPivot)
                .until(this::pivotStallDetected)
                .withTimeout(AlgaeManipulatorConstants.PIVOT_STALL_TIMEOUT_SEC);
    }

    /**
     * Note the side effect of setting {@link #readyToScore} to true when stall detected.
     * 
     * @return true when the pivot motor has stalled.
     */
    private boolean pivotStallDetected() {
        final boolean stalled = this.pivotStallDetection.calculate(
                Math.abs(this.pivot.getStatorCurrent()) > AlgaeManipulatorConstants.PIVOT_STALL_THRESHOLD_AMPS);
        if (stalled) {
            this.readyToScore = true;
        }
        return stalled;
    }

    /**
     * This simply runs the motor for now.
     * 
     * <p>
     * TODO add retract arm (never in old code base). Not sure how to trigger it. May the may have algae state? May need
     * different arm stall sensitivity or just time this particular retraction.
     * 
     * @return a command to run the wheel to remove algae from the reef.
     */
    private Command removeAlgaeFromReef() {
        return this
                .startEnd(
                        () -> this.setWheelDutyCycle(AlgaeManipulatorConstants.INTAKE_DUTY_CYCLE),
                        this::stopWheel)
                .until(() -> CoralState.getCurrentState() == CoralState.MAY_HAVE_ALGAE);
    }

    /**
     * Returns a command to retract the arm (move up or inward) via the pivot.
     * 
     * @return a command to retract the algae arm.
     */
    private Command retractArm() {
        return this
                .startEnd(
                        () -> this.setPivotDutyCycle(AlgaeManipulatorConstants.RETRACT_ARM_DUTY_CYCLE),
                        this::stopPivot)
                .until(this::pivotStallDetected)
                .withTimeout(AlgaeManipulatorConstants.PIVOT_STALL_TIMEOUT_SEC);
    }

    /**
     * @return a command to score algae in to the barge and then retract the arm.
     */
    private Command scoreAlgaeIntoBarge() {
        return this.runOnce(() -> this.setWheelDutyCycle(AlgaeManipulatorConstants.SCORE_DUTY_CYCLE))
                .andThen(Commands.waitSeconds(AlgaeManipulatorConstants.SCORE_DURATION_SEC))
                .andThen(runOnce(this::stopWheel))
                .andThen(this::retractArm);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Manipulator / Arm Pivot Stator Current", this.pivot.getStatorCurrent());
    }
}
