package frc.robot.subsystems.algaemanipulator;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioBusCANIds;

/**
 * The algae manipulator is used to remove algae from the reed and, if held during the dealgae operation, score the
 * algae into the barge. There are commands to deploy and retract the arm, and to run the dealgae/scoring wheel.
 * 
 * <p>
 * TODO The python code for the manipulator has some serious issues. For example, the code and comments disagree on
 * which duty cycle sign pivots the arm up or down. Going with the code here. Also, the "deAlgaefyingCommand" command
 * defined in the python robot container, appears to timeout much to quickly. We should be detecting stall or a slow
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

    /**
     * Creates the algae manipulator subsystem, configures the motors, and creates game piece state bindings.
     */
    public AlgaeManipulator() {
        this.wheel.configSupplyCurrentLimit(AlgaeManipulatorConstants.SUPPLY_CONFIG);
        this.pivot.configSupplyCurrentLimit(AlgaeManipulatorConstants.SUPPLY_CONFIG);

        // The algae manipulator is usually fully stopped.
        this.setDefaultCommand(this.stop());

        // TODO add algae state bindings here and in robot container.
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
        this.setMotorDutyCycle(this.wheel, 0.0);
    }

    /**
     * Utility method to stop the pivot.
     */
    private void stopPivot() {
        this.setMotorDutyCycle(this.pivot, 0.0);
    }

    /**
     * Utility method to set a motor to a particular duty cycle.
     * 
     * @param motor
     *            the motor to receive the duty cycle setting.
     * @param dutyCycle
     *            the duty cycle to set on the motor.
     */
    private void setMotorDutyCycle(final TalonSRX motor, final double dutyCycle) {
        motor.set(TalonSRXControlMode.PercentOutput, dutyCycle);
    }

    /**
     * Returns a command to deploy the arm (move down or outward) via the pivot in order to remove algae from the reef.
     * 
     * @return a command to deploy the algae arm.
     */
    private Command deployArm() {
        return this.startEnd(
                () -> this.setMotorDutyCycle(this.pivot, AlgaeManipulatorConstants.DEPLOY_ARM_DUTY_CYCLE),
                this::stopPivot)
                .until(this::pivotStallDetected);
    }

    private boolean pivotStallDetected() {
        return this.pivotStallDetection.calculate(
                Math.abs(this.pivot.getStatorCurrent()) > AlgaeManipulatorConstants.PIVOT_STALL_THRESHOLD_AMPS);
    }

    /**
     * This simply runs the motor for now.
     * 
     * @return a command to run the wheel to remove algae from the reef.
     */
    private Command removeAlgaeFromReef() {
        return this.startEnd(
                () -> this.setMotorDutyCycle(this.wheel, AlgaeManipulatorConstants.INTAKE_DUTY_CYCLE),
                this::stopWheel);
    }

    /**
     * Returns a command to retract the arm (move up or inward) via the pivot.
     * 
     * @return a command to retract the algae arm.
     */
    private Command retractArm() {
        return this.startEnd(
                () -> this.setMotorDutyCycle(this.pivot, AlgaeManipulatorConstants.RETRACT_ARM_DUTY_CYCLE),
                this::stopPivot)
                .until(this::pivotStallDetected);
    }

    /**
     * @return a command to score algae in to the barge and then retract the arm.
     */
    private Command scoreAlgaeIntoBarge() {
        return this.runOnce(() -> this.setMotorDutyCycle(this.wheel, AlgaeManipulatorConstants.SCORE_DUTY_CYCLE))
                .andThen(Commands.waitSeconds(AlgaeManipulatorConstants.SCORE_DURATION_SEC))
                .andThen(runOnce(this::stopWheel))
                .andThen(this::retractArm);
    }
}
