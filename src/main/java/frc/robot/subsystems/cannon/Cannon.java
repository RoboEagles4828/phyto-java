package frc.robot.subsystems.cannon;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RioBusCANIds;
import frc.robot.game.CoralState;
import frc.robot.game.ElevatedLevel;
import frc.robot.game.CoralLevel;

/**
 * The Cannon is used to intake coral from the hopper and to score it on the reef.
 */
public class Cannon extends SubsystemBase {
    /** Motor to drive wheels on the left side of the cannon. */
    private final TalonSRX leftsideWheels = new TalonSRX(RioBusCANIds.CANNON_LEFT_MOTOR_CANID);
    /** Motor to drive wheels on the right side of the cannon. */
    private final TalonSRX rightsideWheels = new TalonSRX(RioBusCANIds.CANNON_RIGHT_MOTOR_CANID);
    /** Stall during intake debounce for coral loaded detection. */
    private final Debouncer intakeStallDetection = new Debouncer(
            CannonConstants.INTAKE_STALL_DEBOUNCE_SEC,
            DebounceType.kBoth);

    /**
     * Creates the cannon subsystem, configures the motors, and creates game piece state bindings.
     */
    public Cannon() {
        this.leftsideWheels.configSupplyCurrentLimit(CannonConstants.SUPPLY_CONFIG);
        this.leftsideWheels.setInverted(true);
        this.rightsideWheels.configSupplyCurrentLimit(CannonConstants.SUPPLY_CONFIG);
        this.rightsideWheels.setInverted(false);

        // The cannon is usually stopped.
        this.setDefaultCommand(this.stop());
        // Intake until coral detected or the coral state changes from intake.
        CoralState.INTAKE.getTrigger().whileTrue(this.intake());
        // Score coral while coral state is scoring.
        CoralState.SCORE.getTrigger().whileTrue(this.score());
    }

    /**
     * @return a new command to stop the hopper feed wheels.
     */
    private Command stop() {
        // Not runOnce to keep the command running (this is the default).
        // Not run to avoid uneeded CAN bus traffic.
        return this.startEnd(
                () -> this.setCannonSpeeds(0.0),
                () -> this.setCannonSpeeds(0.0));
    }

    /**
     * If a coral is successfully intaken, the coral state is changed to {@link CoralState#CARRY}.
     * 
     * @return a command to intake coral until detected.
     */
    private Command intake() {
        return this.run(() -> setCannonSpeeds(CannonConstants.INTAKE_DUTY_CYCLE))
                .withDeadline(this.stopIntaking())
                .finallyDo(this::postIntakeState);
    }

    /**
     * Used by the intake command to set the next coral state depending on success or interruption.
     * 
     * @param interrupted
     *            true if the intake command was interrupted, or false if it finished with a good load.
     */
    private void postIntakeState(final boolean interrupted) {
        if (interrupted) {
            CoralState.setCurrentState(CoralState.EMPTY);
        } else {
            CoralState.setCurrentState(CoralState.CARRY);
        }
    }

    /**
     * Returns a command that ends when it is appropriate to end the intaking action. Right now it is a stall but could
     * easily be changed to a beam break or other.
     */
    private Command stopIntaking() {
        return new WaitUntilCommand(() -> this.intakeStallDetection.calculate(
                Math.abs(this.rightsideWheels.getStatorCurrent()) > CannonConstants.INTAKE_STALL_THRESHOLD_AMPS));
    }

    /**
     * The coral state is changed to {@link CoralState#EMPTY}.
     * 
     * @return a command to score coral until the stop scoring condition is detected.
     */
    private Command score() {
        return this.run(this::setScoreSpeeds)
                .withDeadline(this.stopScoring())
                .andThen(this.runOnce(() -> CoralState.setCurrentState(CoralState.EMPTY)));
    }

    /**
     * Returns a command that ends when it is appropriate to end the scoring action. Right now it is a time out but
     * could easily be changed to a beam break or other.
     */
    private Command stopScoring() {
        return new WaitCommand(1.0);
    }

    /**
     * Sets the motor speeds for scoring depending on target reef level.
     */
    private void setScoreSpeeds() {
        if (ElevatedLevel.getCurrentLevel() == CoralLevel.L1) {
            setCannonSpeeds(CannonConstants.LEFT_SCORE_L1_DUTY_CYCLE, CannonConstants.RIGHT_SCORE_L1_DUTY_CYCLE);
        } else {
            setCannonSpeeds(CannonConstants.LEFT_SCORE_STRAIGHT_DUTY_CYCLE,
                    CannonConstants.RIGHT_SCORE_STRAIGHT_DUTY_CYCLE);
        }
    }

    /**
     * @param bothSides
     *            the percent power for both sides of the cannon.
     */
    private void setCannonSpeeds(final double bothSides) {
        this.setCannonSpeeds(bothSides, bothSides);
    }

    /**
     * @param left
     *            the left side percent power.
     * @param right
     *            the right side percent power.
     */
    private void setCannonSpeeds(final double left, final double right) {
        this.leftsideWheels.set(TalonSRXControlMode.PercentOutput, left);
        this.rightsideWheels.set(TalonSRXControlMode.PercentOutput, right);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Cannon / Left Stator Current", this.leftsideWheels.getStatorCurrent());
        SmartDashboard.putNumber("Cannon / Right Stator Current", this.rightsideWheels.getStatorCurrent());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
