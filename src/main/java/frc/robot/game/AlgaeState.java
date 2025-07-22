package frc.robot.game;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Defines the various states an algae game piece can enter while the robot interacts with it.
 */
public enum AlgaeState {
    /** There is no algae in the robot (that we know of). */
    EMPTY,
    /** The robot will actively intake algae from the reef. */
    INTAKE,
    /** The robot has a properly loaded algae and will carry it. */
    CARRY,
    /** The robot is preparing to score (elevator to proper level). */
    PREPARE_TO_SCORE,
    /** The robot is ready to score its algae. */
    READY_TO_SCORE,
    /** The robot will actively score the algae into the barge. */
    LOAD_BARGE,
    /** An algae is to be removed from the reef without intaking it (drop to floor). */
    REMOVE_FROM_REEF;

    /** The current algae state. We always start a match without an algae. */
    private static AlgaeState currentState = EMPTY;
    /** A trigger for each enumeration value. */
    private final Trigger trigger;

    /**
     * Called to create each enumeration value. A trigger for each value.
     */
    private AlgaeState() {
        this.trigger = new Trigger(() -> getCurrentState() == this);
    }

    /**
     * @return the trigger for this enumeration value.
     */
    public Trigger getTrigger() {
        return this.trigger;
    }

    /**
     * @return the current algae state (never null).
     */
    public static AlgaeState getCurrentState() {
        return currentState;
    }

    /**
     * Sets the next algae state.
     * 
     * @param nextState
     *            the state to set. If null, the state does not change.
     */
    public static void setCurrentState(final AlgaeState nextState) {
        if (nextState != null) {
            currentState = nextState;
        }
    }
}
